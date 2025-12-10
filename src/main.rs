use canparse::pgn::{ParseMessage, PgnLibrary, SpnDefinition};
use dbc::*;
use slint::{ModelRc, Timer, TimerMode, VecModel};
use std::collections::HashMap;
use std::fs::File;
use std::io;
use std::io::{Read, Write};
use std::path::Path;
use std::time::Duration;
use tokio::fs::{create_dir_all, OpenOptions};
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio_serial::SerialPortBuilderExt;

use crate::diagnostics::{FlashLamp, Lamp, FMI};

slint::include_modules!();

mod dbc;
mod diagnostics;

#[derive(Debug)]
pub struct CANFrame {
    pub id: u16,
    pub dlc: u8,
    pub data: [u8; 8],
}

/// Represents an extended CAN frame with a 29‑bit identifier.
/// The stored `id` is the raw CAN id without the extended flag;
/// when framing, the flag (0x80000000) is added.
#[derive(Debug)]
pub struct ExtendedCANFrame {
    pub id: u32, // lower 29 bits (the extended flag is added during serialization)
    pub dlc: u8,
    pub data: [u8; 8],
}

impl CANFrame {
    /// Create a standard (11‑bit) CANFrame from a byte slice.
    /// The expected format is:
    ///   Start marker (0xAA)
    ///   1‑byte header: high nibble = 0xC, low nibble = DLC
    ///   2 bytes for the 11‑bit CAN ID (low 8 bits then upper 3 bits)
    ///   `dlc` data bytes
    ///   End marker (0x55)
    fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 4 {
            return None; // Not enough data.
        }

        let header = bytes[0];
        if (header & 0xF0) != 0xC0 {
            return None; // Incorrect header for standard CAN.
        }
        let dlc = header & 0x0F;
        if dlc > 8 {
            return None;
        }

        // Next two bytes hold the 11‑bit CAN ID:
        // lower 8 bits in bytes[1] and upper 3 bits (in lower nibble of bytes[2]).
        let id_low = bytes[1] as u16;
        let id_high = (bytes[2] as u16) & 0x07;
        let read_id = (id_high << 8) | id_low;

        if bytes.len() < 3 + dlc as usize {
            return None;
        }
        let mut data = [0u8; 8];
        for i in 0..dlc as usize {
            data[i] = bytes[3 + i];
        }

        Some(CANFrame {
            id: read_id,
            dlc,
            data,
        })
    }

    /// Serialize this standard CAN frame into a byte vector with start (0xAA)
    /// and end (0x55) markers.
    fn to_bytes(&self) -> Vec<u8> {
        let mut payload = Vec::with_capacity(3 + self.dlc as usize);

        // Header: high nibble 0xC, low nibble = DLC.
        let header = 0xC0 | (self.dlc & 0x0F);
        payload.push(header);

        // Next two bytes: lower 8 bits of the ID, then the upper 3 bits.
        payload.push((self.id & 0xFF) as u8);
        payload.push(((self.id >> 8) & 0x07) as u8);

        // Data bytes (only the first `dlc` bytes are sent).
        for i in 0..self.dlc as usize {
            payload.push(self.data[i]);
        }

        // Wrap with start (0xAA) and end (0x55) markers.
        let mut frame = Vec::with_capacity(payload.len() + 2);
        frame.push(0xAA);
        frame.extend_from_slice(&payload);
        frame.push(0x55);
        frame
    }
}

impl ExtendedCANFrame {
    /// Parse an extended CAN frame from a byte slice.
    /// Expected format:
    ///   Start marker (0xAA)
    ///   1‑byte header: high nibble = 0xE, low nibble = DLC
    ///   4 bytes for the CAN ID (in little‑endian order)
    ///   `dlc` data bytes
    ///   End marker (0x55)
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 5 {
            return None;
        }
        let header = bytes[0];
        if (header & 0xF0) != 0xE0 {
            return None;
        }
        let dlc = header & 0x0F;
        if dlc > 8 || bytes.len() < 1 + 4 + dlc as usize {
            return None;
        }

        // Read the 4‑byte CAN ID (little‑endian).
        let raw_id = u32::from_le_bytes([bytes[1], bytes[2], bytes[3], bytes[4]]);
        // Add the extended flag.
        let id = raw_id | 0x80000000;

        let mut data = [0u8; 8];
        for i in 0..dlc as usize {
            data[i] = bytes[5 + i];
        }

        Some(ExtendedCANFrame { id, dlc, data })
    }

    /// Serialize this extended frame using our custom framing:
    ///   Start marker (0xAA)
    ///   Header: 0xE0 | (dlc & 0x0F)
    ///   4 bytes: CAN ID (little‑endian) with the extended flag added
    ///   `dlc` data bytes
    ///   End marker (0x55)
    pub fn to_bytes(&self) -> Vec<u8> {
        let header = 0xE0 | (self.dlc & 0x0F);
        // Only lower 29 bits of id are valid; add the extended flag.
        let full_id = 0x80000000 | (self.id & 0x1FFF_FFFF);
        let mut payload = Vec::with_capacity(1 + 4 + self.dlc as usize);
        payload.push(header);
        payload.extend_from_slice(&full_id.to_le_bytes());
        for i in 0..self.dlc as usize {
            payload.push(self.data[i]);
        }
        let mut frame = Vec::with_capacity(payload.len() + 2);
        frame.push(0xAA);
        frame.extend_from_slice(&payload);
        frame.push(0x55);
        frame
    }
}

#[derive(Debug, Copy, Clone)]
struct ProprietaryB5States {
    pub brake_switch_rear: u8,
    pub kickstand_switch: u8,
    pub brake_switch_front: u8,
}

impl ProprietaryB5States {
    pub fn new() -> Self {
        Self {
            brake_switch_rear: 0,
            kickstand_switch: 0,
            brake_switch_front: 0,
        }
    }
    pub fn toggle_brake_switch_rear(&mut self) {
        if self.brake_switch_rear == 0 {
            self.brake_switch_rear = 1;
        } else {
            self.brake_switch_rear = 0;
        }
    }
    pub fn toggle_brake_switch_front(&mut self) {
        if self.brake_switch_front == 0 {
            self.brake_switch_front = 1;
        } else {
            self.brake_switch_front = 0;
        }
    }
    pub fn toggle_kickstand_switch(&mut self) {
        if self.kickstand_switch == 0 {
            self.kickstand_switch = 1;
        } else {
            self.kickstand_switch = 0;
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct ProprietarySwitchedPowerInputStates {
    pub input_0: u8,
    pub input_1: u8,
    pub input_2: u8,
    pub input_3: u8,
    pub input_4: u8,
    pub input_5: u8,
    pub input_6: u8,
    pub input_7: u8,
    pub input_8: u8,
    pub input_9: u8,
    pub input_10: u8,
    pub high_current_0: u8,
    pub high_current_1: u8,
}

impl ProprietarySwitchedPowerInputStates {
    pub fn new() -> Self {
        Self {
            input_0: 0,
            input_1: 0,
            input_2: 0,
            input_3: 0,
            input_4: 0,
            input_5: 0,
            input_6: 0,
            input_7: 0,
            input_8: 0,
            input_9: 0,
            input_10: 0,
            high_current_0: 0,
            high_current_1: 0,
        }
    }

    pub fn toggle_input_0(&mut self) {
        if self.input_0 == 0 {
            self.input_0 = 1;
        } else {
            self.input_0 = 0;
        }
    }

    pub fn toggle_input_1(&mut self) {
        if self.input_1 == 0 {
            self.input_1 = 1;
        } else {
            self.input_1 = 0;
        }
    }

    pub fn toggle_input_2(&mut self) {
        if self.input_2 == 0 {
            self.input_2 = 1;
        } else {
            self.input_2 = 0;
        }
    }

    pub fn toggle_input_3(&mut self) {
        if self.input_3 == 0 {
            self.input_3 = 1;
        } else {
            self.input_3 = 0;
        }
    }

    pub fn toggle_input_4(&mut self) {
        if self.input_4 == 0 {
            self.input_4 = 1;
        } else {
            self.input_4 = 0;
        }
    }

    pub fn toggle_input_5(&mut self) {
        if self.input_5 == 0 {
            self.input_5 = 1;
        } else {
            self.input_5 = 0;
        }
    }

    pub fn toggle_input_6(&mut self) {
        if self.input_6 == 0 {
            self.input_6 = 1;
        } else {
            self.input_6 = 0;
        }
    }

    pub fn toggle_input_7(&mut self) {
        if self.input_7 == 0 {
            self.input_7 = 1;
        } else {
            self.input_7 = 0;
        }
    }

    pub fn toggle_input_8(&mut self) {
        if self.input_8 == 0 {
            self.input_8 = 1;
        } else {
            self.input_8 = 0;
        }
    }

    pub fn toggle_input_9(&mut self) {
        if self.input_9 == 0 {
            self.input_9 = 1;
        } else {
            self.input_9 = 0;
        }
    }

    pub fn toggle_input_10(&mut self) {
        if self.input_10 == 0 {
            self.input_10 = 1;
        } else {
            self.input_10 = 0;
        }
    }

    pub fn toggle_high_current_0(&mut self) {
        if self.high_current_0 == 0 {
            self.high_current_0 = 1;
        } else {
            self.high_current_0 = 0;
        }
    }

    pub fn toggle_high_current_1(&mut self) {
        if self.high_current_1 == 0 {
            self.high_current_1 = 1;
        } else {
            self.high_current_1 = 0;
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub struct HighVoltageEnergyStorageSystemData1 {
    /// kW
    pub available_discharge_power: f32,
    /// kW
    pub available_charge_power: f32,
    /// V
    pub voltage_level: f32,
    /// A (signed)
    pub current: f32,
}

impl HighVoltageEnergyStorageSystemData1 {
    const SCALE: f32 = 0.05;

    pub fn new() -> Self {
        Self {
            available_discharge_power: 0.0,
            available_charge_power: 0.0,
            voltage_level: 0.0,
            current: 0.0,
        }
    }

    // ----- Setters in physical units -----

    pub fn set_available_discharge_power(&mut self, value_kw: f32) {
        // DBC: [0 | 3212.75] kW
        self.available_discharge_power = value_kw.clamp(0.0, 3212.75);
    }

    pub fn set_available_charge_power(&mut self, value_kw: f32) {
        // DBC: [0 | 3212.75] kW
        self.available_charge_power = value_kw.clamp(0.0, 3212.75);
    }

    pub fn set_voltage_level(&mut self, value_v: f32) {
        // DBC: [0 | 3212.75] V
        self.voltage_level = value_v.clamp(0.0, 3212.75);
    }

    pub fn set_current(&mut self, value_a: f32) {
        // DBC: [-1600 | 1612.75] A
        self.current = value_a.clamp(-1600.0, 1612.75);
    }

    // ----- CAN encode/decode with scaling -----
    // Use these where you actually touch the CAN frame.

    pub fn from_can(data: &[u8; 8]) -> Self {
        let discharge_raw = u16::from_le_bytes([data[0], data[1]]);
        let charge_raw = u16::from_le_bytes([data[2], data[3]]);
        let voltage_raw = u16::from_le_bytes([data[4], data[5]]);
        let current_raw = i16::from_le_bytes([data[6], data[7]]);

        Self {
            available_discharge_power: (discharge_raw as f32) * Self::SCALE,
            available_charge_power: (charge_raw as f32) * Self::SCALE,
            voltage_level: (voltage_raw as f32) * Self::SCALE,
            current: (current_raw as f32) * Self::SCALE,
        }
    }

    pub fn to_can(&self) -> [u8; 8] {
        let discharge = Self::encode_u16(self.available_discharge_power, 0.0, 3212.75);
        let charge = Self::encode_u16(self.available_charge_power, 0.0, 3212.75);
        let voltage = Self::encode_u16(self.voltage_level, 0.0, 3212.75);
        let current = Self::encode_i16(self.current, -1600.0, 1612.75);

        let mut data = [0u8; 8];
        data[0..2].copy_from_slice(&discharge.to_le_bytes()); // 0|16
        data[2..4].copy_from_slice(&charge.to_le_bytes()); // 16|16
        data[4..6].copy_from_slice(&voltage.to_le_bytes()); // 32|16
        data[6..8].copy_from_slice(&current.to_le_bytes()); // 48|16 (signed)
        data
    }

    fn encode_u16(value: f32, min: f32, max: f32) -> u16 {
        let clamped = value.clamp(min, max);
        let raw = (clamped / Self::SCALE).round();
        raw as u16
    }

    fn encode_i16(value: f32, min: f32, max: f32) -> i16 {
        let clamped = value.clamp(min, max);
        let raw = (clamped / Self::SCALE).round();
        raw as i16
    }
}

// ======================================================================
// High_Voltage_Energy_Storage_System_Data_2
// BO_ 2565902846
// ======================================================================

#[derive(Debug, Copy, Clone)]
pub struct HighVoltageEnergyStorageSystemData2 {
    /// HVESS_Fast_Update_State_of_Charge [%]
    pub fast_update_state_of_charge: f32,
    /// HVESS_Highest_Cell_Voltage [V]
    pub highest_cell_voltage: f32,
    /// HVESS_Lowest_Cell_Voltage [V]
    pub lowest_cell_voltage: f32,
    /// HVESS_Cell_Voltage_Differential_Status [0..15] (4-bit)
    pub cell_voltage_differential_status: u8,
}

impl HighVoltageEnergyStorageSystemData2 {
    const SOC_SCALE: f32 = 0.0015625; // 1/640
    const VOLT_SCALE: f32 = 0.001;

    pub fn new() -> Self {
        Self {
            fast_update_state_of_charge: 0.0,
            highest_cell_voltage: 0.0,
            lowest_cell_voltage: 0.0,
            cell_voltage_differential_status: 0,
        }
    }

    // -------- setters in physical units --------

    pub fn set_fast_update_state_of_charge(&mut self, soc_percent: f32) {
        // [0 | 100.398] %
        self.fast_update_state_of_charge = soc_percent.clamp(0.0, 100.398);
    }

    pub fn set_highest_cell_voltage(&mut self, v: f32) {
        // [0 | 64.225] V
        self.highest_cell_voltage = v.clamp(0.0, 64.225);
    }

    pub fn set_lowest_cell_voltage(&mut self, v: f32) {
        // [0 | 64.225] V
        self.lowest_cell_voltage = v.clamp(0.0, 64.225);
    }

    pub fn set_cell_voltage_differential_status(&mut self, status: u8) {
        // 4 bits @ 48|4 -> 0..15
        self.cell_voltage_differential_status = status.min(0x0F);
    }

    // -------- encode/decode with DBC scaling --------

    pub fn from_can(data: &[u8; 8]) -> Self {
        let soc_raw = u16::from_le_bytes([data[0], data[1]]);
        let high_v_raw = u16::from_le_bytes([data[2], data[3]]);
        let low_v_raw = u16::from_le_bytes([data[4], data[5]]);
        let status = data[6] & 0x0F; // 48|4

        Self {
            fast_update_state_of_charge: soc_raw as f32 * Self::SOC_SCALE,
            highest_cell_voltage: high_v_raw as f32 * Self::VOLT_SCALE,
            lowest_cell_voltage: low_v_raw as f32 * Self::VOLT_SCALE,
            cell_voltage_differential_status: status,
        }
    }

    pub fn to_can(&self) -> [u8; 8] {
        let soc_raw = Self::encode_u16(
            self.fast_update_state_of_charge,
            0.0,
            100.398,
            Self::SOC_SCALE,
        );
        let high_v_raw = Self::encode_u16(self.highest_cell_voltage, 0.0, 64.225, Self::VOLT_SCALE);
        let low_v_raw = Self::encode_u16(self.lowest_cell_voltage, 0.0, 64.225, Self::VOLT_SCALE);
        let status = self.cell_voltage_differential_status & 0x0F;

        let mut data = [0u8; 8];

        // HVESS_Fast_Update_State_of_Charge : 0|16
        data[0..2].copy_from_slice(&soc_raw.to_le_bytes());
        // HVESS_Highest_Cell_Voltage : 16|16
        data[2..4].copy_from_slice(&high_v_raw.to_le_bytes());
        // HVESS_Lowest_Cell_Voltage : 32|16
        data[4..6].copy_from_slice(&low_v_raw.to_le_bytes());
        // HVESS_Cell_Voltage_Differential_Status : 48|4
        data[6] = (data[6] & !0x0F) | status;

        data
    }

    #[inline]
    fn encode_u16(value: f32, min: f32, max: f32, scale: f32) -> u16 {
        let clamped = value.clamp(min, max);
        let raw = (clamped / scale).round();
        raw as u16
    }
}

// ======================================================================
// High_Voltage_Energy_Storage_System_Data_3
// BO_ 2565903102
// ======================================================================

#[derive(Debug, Copy, Clone)]
pub struct HighVoltageEnergyStorageSystemData3 {
    /// HVESS_Highest_Cell_Temperature [°C]
    pub highest_cell_temperature: f32,
    /// HVESS_Lowest_Cell_Temperature [°C]
    pub lowest_cell_temperature: f32,
    /// HVESS_Average_Cell_Temperature [°C]
    pub average_cell_temperature: f32,
    /// High_Voltage_Energy_Storage_System_Data_3Sig492 (2-bit status)
    pub status: u8,
}

impl HighVoltageEnergyStorageSystemData3 {
    const TEMP_SCALE: f32 = 0.03125;
    const TEMP_OFFSET: f32 = -273.0;

    pub fn new() -> Self {
        Self {
            highest_cell_temperature: 0.0,
            lowest_cell_temperature: 0.0,
            average_cell_temperature: 0.0,
            status: 0,
        }
    }

    // -------- setters in physical units --------

    pub fn set_highest_cell_temperature(&mut self, temp_c: f32) {
        self.highest_cell_temperature = temp_c.clamp(-273.0, 1734.97);
    }

    pub fn set_lowest_cell_temperature(&mut self, temp_c: f32) {
        self.lowest_cell_temperature = temp_c.clamp(-273.0, 1734.97);
    }

    pub fn set_average_cell_temperature(&mut self, temp_c: f32) {
        self.average_cell_temperature = temp_c.clamp(-273.0, 1734.97);
    }

    pub fn set_status(&mut self, status: u8) {
        // 48|2 -> 0..3
        self.status = status.min(0x03);
    }

    // -------- encode/decode with DBC scaling --------

    pub fn from_can(data: &[u8; 8]) -> Self {
        let high_raw = u16::from_le_bytes([data[0], data[1]]);
        let low_raw = u16::from_le_bytes([data[2], data[3]]);
        let avg_raw = u16::from_le_bytes([data[4], data[5]]);
        let status = data[6] & 0x03; // 48|2

        Self {
            highest_cell_temperature: Self::decode_temp(high_raw),
            lowest_cell_temperature: Self::decode_temp(low_raw),
            average_cell_temperature: Self::decode_temp(avg_raw),
            status,
        }
    }

    pub fn to_can(&self) -> [u8; 8] {
        let high_raw = Self::encode_temp(self.highest_cell_temperature);
        let low_raw = Self::encode_temp(self.lowest_cell_temperature);
        let avg_raw = Self::encode_temp(self.average_cell_temperature);
        let status = self.status & 0x03;

        let mut data = [0u8; 8];

        // HVESS_Highest_Cell_Temperature : 0|16
        data[0..2].copy_from_slice(&high_raw.to_le_bytes());
        // HVESS_Lowest_Cell_Temperature : 16|16
        data[2..4].copy_from_slice(&low_raw.to_le_bytes());
        // HVESS_Average_Cell_Temperature : 32|16
        data[4..6].copy_from_slice(&avg_raw.to_le_bytes());
        // High_Voltage_Energy_Storage_System_Data_3Sig492 : 48|2
        data[6] = (data[6] & !0x03) | status;

        data
    }

    #[inline]
    fn decode_temp(raw: u16) -> f32 {
        (raw as f32) * Self::TEMP_SCALE + Self::TEMP_OFFSET
    }

    #[inline]
    fn encode_temp(temp_c: f32) -> u16 {
        let clamped = temp_c.clamp(-273.0, 1734.97);
        let raw = ((clamped - Self::TEMP_OFFSET) / Self::TEMP_SCALE).round();
        raw as u16
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ActiveDiagnosticTroubleCodesPDIO {
    pub protect_lamp: Lamp,
    pub amber_warning: Lamp,
    pub red_stop: Lamp,
    pub malfunction: Lamp,
    pub protect_lamp_flash: FlashLamp,
    pub amber_warning_flash: FlashLamp,
    pub red_stop_flash: FlashLamp,
    pub malfunction_flash: FlashLamp,
    pub source_spn: u32,       // 19 bits
    pub failure_mode: FMI,     // 5 bits
    pub occurrence_count: u8,  // 7 bits
    pub conv_method_old: bool, // 1 bit
    pub reserved: u16,         // 16 bits
}

impl ActiveDiagnosticTroubleCodesPDIO {
    pub fn new() -> Self {
        Self {
            protect_lamp: Lamp::Off,
            amber_warning: Lamp::Off,
            red_stop: Lamp::Off,
            malfunction: Lamp::Off,
            protect_lamp_flash: FlashLamp::DoNotFlash,
            amber_warning_flash: FlashLamp::DoNotFlash,
            red_stop_flash: FlashLamp::DoNotFlash,
            malfunction_flash: FlashLamp::DoNotFlash,
            source_spn: 0,
            failure_mode: FMI::DataValidAboveNormalOpRange,
            occurrence_count: 0,
            conv_method_old: false,
            reserved: 0xFFFF,
        }
    }

    pub fn toggle_protect_lamp(&mut self) {
        self.protect_lamp = match self.protect_lamp {
            Lamp::Off => Lamp::On,
            Lamp::On => Lamp::Off,
            _ => Lamp::Off,
        };
    }

    pub fn toggle_amber_warning(&mut self) {
        self.amber_warning = match self.amber_warning {
            Lamp::Off => Lamp::On,
            Lamp::On => Lamp::Off,
            _ => Lamp::Off,
        };
    }

    pub fn toggle_red_stop(&mut self) {
        self.red_stop = match self.red_stop {
            Lamp::Off => Lamp::On,
            Lamp::On => Lamp::Off,
            _ => Lamp::Off,
        };
    }

    pub fn toggle_malfunction(&mut self) {
        self.malfunction = match self.malfunction {
            Lamp::Off => Lamp::On,
            Lamp::On => Lamp::Off,
            _ => Lamp::Off,
        };
    }

    pub fn toggle_protect_lamp_flash(&mut self) {
        self.protect_lamp_flash = match self.protect_lamp_flash {
            FlashLamp::DoNotFlash => FlashLamp::SlowFlash,
            FlashLamp::SlowFlash => FlashLamp::DoNotFlash,
            FlashLamp::FastFlash => FlashLamp::DoNotFlash,
            _ => FlashLamp::DoNotFlash,
        };
    }

    pub fn toggle_amber_warning_flash(&mut self) {
        self.amber_warning_flash = match self.amber_warning_flash {
            FlashLamp::DoNotFlash => FlashLamp::SlowFlash,
            FlashLamp::SlowFlash => FlashLamp::DoNotFlash,
            FlashLamp::FastFlash => FlashLamp::DoNotFlash,
            _ => FlashLamp::DoNotFlash,
        };
    }

    pub fn toggle_red_stop_flash(&mut self) {
        self.red_stop_flash = match self.red_stop_flash {
            FlashLamp::DoNotFlash => FlashLamp::SlowFlash,
            FlashLamp::SlowFlash => FlashLamp::DoNotFlash,
            FlashLamp::FastFlash => FlashLamp::DoNotFlash,
            _ => FlashLamp::DoNotFlash,
        };
    }

    pub fn toggle_malfunction_flash(&mut self) {
        self.malfunction_flash = match self.malfunction_flash {
            FlashLamp::DoNotFlash => FlashLamp::SlowFlash,
            FlashLamp::SlowFlash => FlashLamp::DoNotFlash,
            FlashLamp::FastFlash => FlashLamp::DoNotFlash,
            _ => FlashLamp::DoNotFlash,
        };
    }
}

#[tokio::main]
async fn main() -> io::Result<()> {
    let port_name = r"\\.\COM3"; // Adjust for your system.
    let baud_rate = 115200;

    // Load your DBC file (if needed).
    let _lib = PgnLibrary::from_dbc_file("./src/Riki_j1939.dbc").unwrap();

    /*
        // Open the serial port.
        let serial_port = tokio_serial::new(port_name, baud_rate)
            .timeout(Duration::from_millis(100))
            .open_native_async()?;

        // Split into reader and writer.
        let (mut reader, mut writer) = tokio::io::split(serial_port);
    */
    // Task for reading incoming frames.

    let reader_task = tokio::spawn(async move {
        let mut buffer: Vec<u8> = Vec::new();
        let mut read_buf = [0u8; 128];
        /*
        loop {
            match reader.read(&mut read_buf).await {
                Ok(n) if n == 0 => continue,
                Ok(n) => {
                    buffer.extend_from_slice(&read_buf[..n]);
                    println!("Raw buffer in hex: {:02X?}", buffer);

                    // Process frames delimited by 0xAA ... 0x55.
                    // Also accept back-to-back 0xAA as boundaries.
                    loop {
                        // Find start marker
                        let Some(start) = buffer.iter().position(|&b| b == 0xAA) else {
                            // No start marker at all: clear buffer (noise) and wait for more bytes
                            buffer.clear();
                            break;
                        };

                        // Keep only the potential frame (drop leading noise before start)
                        if start > 0 {
                            buffer.drain(..start);
                        }

                        // Now buffer[0] == 0xAA (start)
                        if buffer.len() <= 1 {
                            // Incomplete (only start byte so far), wait for more
                            break;
                        }

                        // Examine bytes after the start marker
                        let rest = &buffer[1..];

                        // Earliest 0x55 (end) and next 0xAA (start of a new frame)
                        let rel_end55 = rest.iter().position(|&b| b == 0x55);
                        let rel_next_aa = rest.iter().position(|&b| b == 0xAA);

                        // Decide frame end (exclusive) and how much to drain from buffer
                        let (end_exclusive, drain_upto) = match (rel_end55, rel_next_aa) {
                            // Next AA appears before any 55 -> end right before that AA, keep that AA for next loop
                            (Some(p55), Some(paa)) if paa < p55 => (1 + paa, 1 + paa),
                            // Found a 55 (and either no AA or AA is after 55) -> consume up to and including 55
                            (Some(p55), _) => (1 + p55, 1 + p55 + 1),
                            // No 55, but a next AA -> end before that AA, keep AA
                            (None, Some(paa)) => (1 + paa, 1 + paa),
                            // Neither 55 nor AA yet -> incomplete frame; keep from start and wait for more
                            (None, None) => break,
                        };

                        // frame_bytes are between AA and the chosen boundary (exclusive)
                        let frame_bytes = &buffer[1..end_exclusive];

                        if frame_bytes.is_empty() {
                            // Malformed/empty; drop the start byte to avoid infinite loop
                            buffer.drain(..1);
                            continue;
                        }

                        // Try parsing as Extended, then Standard
                        if let Some(xframe) = ExtendedCANFrame::from_bytes(frame_bytes) {
                            println!(
                                "Received Extended CAN Frame: ID=0x{:08X}, DLC={}, Data={:02X?}",
                                xframe.id,
                                xframe.dlc,
                                &xframe.data[..xframe.dlc as usize]
                            );
                        } else if let Some(sframe) = CANFrame::from_bytes(frame_bytes) {
                            println!(
                                "Received Standard CAN Frame: ID={}, DLC={}, Data={:02X?}",
                                sframe.id,
                                sframe.dlc,
                                &sframe.data[..sframe.dlc as usize]
                            );
                            if (100..110).contains(&sframe.id) {
                                handle_expander(
                                    sframe.id.into(),
                                    &sframe.data[..sframe.dlc as usize],
                                )
                                .await;
                            }
                        } else {
                            // Unrecognized; drop the start and try again
                            buffer.drain(..1);
                            continue;
                        }

                        // Remove processed bytes:
                        // - If boundary was next AA, we preserved that AA (drain_upto excludes it).
                        // - If boundary was 55, we consumed it as well.
                        buffer.drain(..drain_upto);

                        // Continue inner loop to look for more frames in the current buffer
                    }
                }
                Err(e) => {
                    eprintln!("Error reading from serial port: {:?}", e);
                    break;
                }
            }

        }
        */
    });

    // Task for handling keyboard input and sending frames.
    let writer_task = tokio::spawn(async move {
        let mut toggle_states: HashMap<u8, bool> = HashMap::new();
        let mut proprietary_b5_states: ProprietaryB5States = ProprietaryB5States::new();
        let mut switched_power_input_states = ProprietarySwitchedPowerInputStates::new();
        let mut hvess2 = HighVoltageEnergyStorageSystemData2::new();
        let mut throttle_value: u8 = 0;
        let mut engine_speed: u16 = 0;
        let stdin = BufReader::new(tokio::io::stdin());
        let mut lines = stdin.lines();
        let mut soc = 50.0; // initial SoC %

        while let Ok(Some(line)) = lines.next_line().await {
            let command = line.trim();
            match command {
                "1" => {
                    // Send a standard CAN frame with ID=0 and data [0, 100].
                    let speed: u16 = 100;
                    let bytes = speed.to_be_bytes();
                    let frame = CANFrame {
                        id: 0,
                        dlc: 2,
                        data: {
                            let mut d = [0u8; 8];
                            d[0] = bytes[0];
                            d[1] = bytes[1];
                            d
                        },
                    };
                    //send_frame\(&mut writer, frame\)\.await;
                }
                "2" => {
                    // Standard CAN frame with ID=1 and data [1].
                    let frame = CANFrame {
                        id: 1,
                        dlc: 1,
                        data: {
                            let mut d = [0u8; 8];
                            d[0] = 1;
                            d
                        },
                    };
                    //send_frame\(&mut writer, frame\)\.await;
                }
                "3" => {
                    // Standard CAN frame with ID=10 and data [0x01, 0xFF].
                    let led_status: u16 = 0x01FF;
                    let bytes = led_status.to_be_bytes();
                    let frame = CANFrame {
                        id: 10,
                        dlc: 2,
                        data: {
                            let mut d = [0u8; 8];
                            d[0] = bytes[0];
                            d[1] = bytes[1];
                            d
                        },
                    };
                    //send_frame\(&mut writer, frame\)\.await;
                }
                "+" => {
                    // Increase throttle_value.
                    if throttle_value < 100 {
                        throttle_value += 5;
                    }
                    let data = encode_proprietary_b4(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 2365522974,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "-" => {
                    // Decrease throttle_value.
                    if throttle_value > 0 {
                        throttle_value -= 5;
                    }
                    let data = encode_proprietary_b4(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 2365522974,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "++" => {
                    // Increase throttle_value.
                    if throttle_value < 85 {
                        throttle_value += 20;
                    }
                    let data = encode_proprietary_b4(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 2365522974,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "--" => {
                    // Decrease throttle_value.
                    if throttle_value > 15 {
                        throttle_value -= 20;
                    }
                    let data = encode_proprietary_b4(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 2365522974,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "+max" => {
                    // Increase throttle_value.
                    throttle_value = 100;

                    let data = encode_proprietary_b4(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 2365522974,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "-max" => {
                    // Decrease throttle_value.
                    throttle_value = 0;

                    let data = encode_proprietary_b4(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 2365522974,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "s+" => {
                    // Increase engine_speed and send an extended frame.
                    if engine_speed < 5500 {
                        engine_speed += 500;
                    }
                    let data = encode_engine_controller1(engine_speed.into(), 0.0, 0.0, 0.0, 0.0);

                    let ext_frame = ExtendedCANFrame {
                        id: 2364539904,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "s-" => {
                    // Decrease engine_speed and send an extended frame.
                    if engine_speed > 0 {
                        engine_speed -= 500;
                    }
                    let data = encode_engine_controller1(engine_speed.into(), 0.0, 0.0, 0.0, 0.0);

                    let ext_frame = ExtendedCANFrame {
                        id: 2364539904,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "b+" => {
                    // Increase SoC
                    if soc < 100.0 {
                        soc += 10.0;
                    }
                    hvess2.set_fast_update_state_of_charge(soc);

                    let data = hvess2.to_can();
                    let ext_frame = ExtendedCANFrame {
                        id: 2565902593,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                    println!("SoC increased to {:.2}%", soc);
                }

                "b-" => {
                    // Decrease SoC
                    if soc > 0.0 {
                        soc -= 10.0;
                    }
                    hvess2.set_fast_update_state_of_charge(soc);

                    let data = hvess2.to_can();
                    let ext_frame = ExtendedCANFrame {
                        id: 2565902593,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                    println!("SoC decreased to {:.2}%", soc);
                }
                "test" => {
                    // Increase engine_speed and send an extended frame.
                    let data = [0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 0x8CEF00FE,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "23" => {
                    //Toggle brake_rear and send Proprietary_b5 message.
                    proprietary_b5_states.toggle_brake_switch_rear();

                    let data = encode_proprietary_b5(proprietary_b5_states);

                    let ext_frame = ExtendedCANFrame {
                        id: 2566849822,
                        dlc: 8,
                        data,
                    };

                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "24" => {
                    //Toggle brake_switch_front and send Proprietary_b5 message.
                    proprietary_b5_states.toggle_brake_switch_front();

                    let data = encode_proprietary_b5(proprietary_b5_states);

                    let ext_frame = ExtendedCANFrame {
                        id: 2566849822,
                        dlc: 8,
                        data,
                    };

                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "25" => {
                    //Toggle kickstand_switch and send Proprietary_b5 message.
                    proprietary_b5_states.toggle_kickstand_switch();

                    let data = encode_proprietary_b5(proprietary_b5_states);

                    let ext_frame = ExtendedCANFrame {
                        id: 2566849822,
                        dlc: 8,
                        data,
                    };

                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "26" => {
                    // High Beam/Passing Beam Signal (HBSI)
                    switched_power_input_states.toggle_input_0();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "27" => {
                    //"Side Stand Switch(SSSI)"
                    switched_power_input_states.toggle_input_1();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "28" => {
                    //"Left Brake Signal(LBSI)"
                    switched_power_input_states.toggle_input_2();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "29" => {
                    //"Turn Left Signal(TLSI)"
                    switched_power_input_states.toggle_input_3();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "30" => {
                    //"Turn Right Signal (TRSI)"
                    switched_power_input_states.toggle_input_4();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "31" => {
                    //"Park Button (PBSI)"
                    switched_power_input_states.toggle_input_5();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "32" => {
                    //"Mode Button(MBSI)"
                    switched_power_input_states.toggle_input_6();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "33" => {
                    //"Reverse Button(RVSI)"
                    switched_power_input_states.toggle_input_7();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "34" => {
                    //"Kıll Switch (KSWI)"
                    switched_power_input_states.toggle_input_8();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "35" => {
                    //"Right Brake Signal(RBSI)"
                    switched_power_input_states.toggle_input_9();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "36" => {
                    //"Hazard Switch(HSWI)"
                    switched_power_input_states.toggle_input_10();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "37" => {
                    switched_power_input_states.toggle_high_current_0();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "38" => {
                    switched_power_input_states.toggle_high_current_1();
                    let data = encode_switched_power_input_status(switched_power_input_states);
                    let ext_frame = ExtendedCANFrame {
                        id: 2566698526,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "40" => {
                    // Absolute overtemperature of the Battery Pack

                    let data = encode_active_diagnostic_trouble_codes_pdio(
                        ActiveDiagnosticTroubleCodesPDIO {
                            protect_lamp: Lamp::Off,
                            amber_warning: Lamp::Off,
                            red_stop: Lamp::On,
                            malfunction: Lamp::Off,
                            protect_lamp_flash: FlashLamp::DoNotFlash,
                            amber_warning_flash: FlashLamp::DoNotFlash,
                            red_stop_flash: FlashLamp::DoNotFlash,
                            malfunction_flash: FlashLamp::DoNotFlash,
                            source_spn: 8121,
                            failure_mode: FMI::DataValidAboveNormalOpRange,
                            occurrence_count: 1,
                            conv_method_old: false,
                            reserved: 0xFF,
                        },
                    );

                    let ext_frame = ExtendedCANFrame {
                        id: 2566834718,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "41" => {
                    // Absolute overtemperature of the Battery Pack

                    let data = encode_active_diagnostic_trouble_codes_pdio(
                        ActiveDiagnosticTroubleCodesPDIO {
                            protect_lamp: Lamp::Off,
                            amber_warning: Lamp::On,
                            red_stop: Lamp::Off,
                            malfunction: Lamp::On,
                            protect_lamp_flash: FlashLamp::DoNotFlash,
                            amber_warning_flash: FlashLamp::DoNotFlash,
                            red_stop_flash: FlashLamp::DoNotFlash,
                            malfunction_flash: FlashLamp::DoNotFlash,
                            source_spn: 8077,
                            failure_mode: FMI::DataValidAboveNormalOpRange,
                            occurrence_count: 1,
                            conv_method_old: false,
                            reserved: 0xFF,
                        },
                    );

                    let ext_frame = ExtendedCANFrame {
                        id: 2566834718,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "42" => {
                    // Absolute overtemperature of the Battery Pack

                    let data = encode_active_diagnostic_trouble_codes_pdio(
                        ActiveDiagnosticTroubleCodesPDIO {
                            protect_lamp: Lamp::Off,
                            amber_warning: Lamp::Off,
                            red_stop: Lamp::On,
                            malfunction: Lamp::Off,
                            protect_lamp_flash: FlashLamp::DoNotFlash,
                            amber_warning_flash: FlashLamp::DoNotFlash,
                            red_stop_flash: FlashLamp::FastFlash,
                            malfunction_flash: FlashLamp::DoNotFlash,
                            source_spn: 5917,
                            failure_mode: FMI::DataValidButBelowNormalOpRangeLeastSevere,
                            occurrence_count: 1,
                            conv_method_old: false,
                            reserved: 0xFF,
                        },
                    );

                    let ext_frame = ExtendedCANFrame {
                        id: 2566834718,
                        dlc: 8,
                        data,
                    };
                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "43" => {
                    // DM3 Clear Fault: Absolute overtemperature of the Battery Pack
                    // Create the DM3Packet and get data bytes
                    let data =
                        encode_dm3packet(5917, FMI::DataValidButBelowNormalOpRangeLeastSevere, 1);

                    // Construct the correct CAN ID for DM3 (PGN 65228)
                    // 0x18FECC00 is PGN 65228 with priority 6 (0x6 << 26)
                    // The source address (last 8 bits) should be set as needed; here using 0x06 as example
                    let can_id = 0x18FECC00 | 0x06; // priority 6 + PGN 0xFECC + source address 0x06

                    let ext_frame = ExtendedCANFrame {
                        id: can_id,
                        dlc: 8,
                        data,
                    };

                    //send_extended_frame(&mut writer, ext_frame).await;
                }
                "44" => {
                    // Absolute overtemperature of the Battery PackUSB

                    let data = encode_active_diagnostic_trouble_codes_pdio(
                        ActiveDiagnosticTroubleCodesPDIO {
                            protect_lamp: Lamp::Off,
                            amber_warning: Lamp::Off,
                            red_stop: Lamp::On,
                            malfunction: Lamp::Off,
                            protect_lamp_flash: FlashLamp::DoNotFlash,
                            amber_warning_flash: FlashLamp::DoNotFlash,
                            red_stop_flash: FlashLamp::FastFlash,
                            malfunction_flash: FlashLamp::DoNotFlash,
                            source_spn: 5917,
                            failure_mode: FMI::DataValidButBelowNormalOpRangeLeastSevere,
                            occurrence_count: 2,
                            conv_method_old: false,
                            reserved: 0xFF,
                        },
                    );

                    let ext_frame = ExtendedCANFrame {
                        id: 2566834718,
                        dlc: 8,
                        data,
                    };
                    //  //send_extended_frame(&mut writer, ext_frame).await;
                }
                "45" => {
                    // Absolute overtemperature of the Battery Pack

                    let data = encode_active_diagnostic_trouble_codes_pdio(
                        ActiveDiagnosticTroubleCodesPDIO {
                            protect_lamp: Lamp::Off,
                            amber_warning: Lamp::Off,
                            red_stop: Lamp::On,
                            malfunction: Lamp::Off,
                            protect_lamp_flash: FlashLamp::DoNotFlash,
                            amber_warning_flash: FlashLamp::DoNotFlash,
                            red_stop_flash: FlashLamp::FastFlash,
                            malfunction_flash: FlashLamp::DoNotFlash,
                            source_spn: 5917,
                            failure_mode: FMI::DataValidButBelowNormalOpRangeLeastSevere,
                            occurrence_count: 3,
                            conv_method_old: false,
                            reserved: 0xFF,
                        },
                    );

                    let ext_frame = ExtendedCANFrame {
                        id: 2566834718,
                        dlc: 8,
                        data,
                    };
                    //   //send_extended_frame(&mut writer, ext_frame).await;
                }

                _ => {
                    // Toggle states for commands 7..21.
                    if let Ok(num) = command.parse::<u8>() {
                        if (7..=22).contains(&num) {
                            let state = toggle_states.entry(num).or_insert(false);
                            *state = !*state;
                            let value = if *state { 1 } else { 0 };
                            let frame = CANFrame {
                                id: num as u16,
                                dlc: 4,
                                data: {
                                    let mut d = [0u8; 8];
                                    d[3] = value;
                                    d
                                },
                            };
                            println!("Toggled id {} to {}", num, value);
                        //    //send_frame\(&mut writer, frame\)\.await;
                        } else {
                            println!("Unknown command: {}", command);
                        }
                    } else {
                        println!("Unknown command: {}", command);
                    }
                }
            }
        }
    });

    let ui_task = tokio::spawn(async move {
        // Write all false values to the txt file at the start of the program
        let mut led_file = match File::create("./led_status.txt") {
            Ok(file) => file,
            Err(_) => {
                eprintln!("Failed to create led_status.txt");
                return Ok(());
            }
        };

        let initial_led_data =
            "false\nfalse\nfalse\nfalse\nfalse\nfalse\nfalse\nfalse\nfalse\nfalse\n";
        if let Err(_) = led_file.write_all(initial_led_data.as_bytes()) {
            eprintln!("Failed to write initial data to led_status.txt");
            return Ok(());
        }

        let ui = AppWindow::new()?;
        let timer = Timer::default();

        let ui_handle = ui.as_weak();
        timer.start(
            TimerMode::Repeated,
            std::time::Duration::from_millis(200),
            move || {
                let ui = ui_handle.unwrap();

                if let Ok(expander) = read_expander_csv(1) {
                    ui.set_derate(expander[15]);
                    ui.set_abs(expander[14]);
                    ui.set_flasher(expander[13]);
                    ui.set_alarmAmber(expander[12]);
                    ui.set_akuBattAlarm(expander[11]);
                    ui.set_motorAlarmMilLamp(expander[10]);
                    ui.set_battery1Indicator(expander[9]);
                    ui.set_leftSignal(expander[8]);
                    ui.set_battery2_60(expander[7]);
                    ui.set_ble(expander[6]);
                    ui.set_battery2_70(expander[5]);
                    ui.set_battery2_80(expander[4]);
                    ui.set_battery2_90(expander[3]);
                    ui.set_battery2_100(expander[2]);
                    ui.set_rightSignal(expander[1]);
                    ui.set_battery2Indicator(expander[0]);
                };

                if let Ok(expander) = read_expander_csv(2) {
                    ui.set_clockMinuteTensC(expander[15]);
                    ui.set_clockMinuteTensD(expander[14]);
                    ui.set_clockMinuteTensE(expander[13]);
                    ui.set_clockMinuteTensG(expander[12]);
                    ui.set_clockMinuteTensF(expander[11]);
                    ui.set_clockMinuteTensA(expander[10]);
                    ui.set_clockMinuteTensB(expander[9]);
                    ui.set_clockMinuteOnesE(expander[8]);
                    ui.set_dateIndicator(expander[7]);
                    ui.set_clockIndicator(expander[6]);
                    ui.set_clockMinuteOnesF(expander[5]);
                    ui.set_clockMinuteOnesA(expander[4]);
                    ui.set_clockMinuteOnesB(expander[3]);
                    ui.set_clockMinuteOnesG(expander[2]);
                    ui.set_clockMinuteOnesC(expander[1]);
                    ui.set_clockMinuteOnesD(expander[0]);
                };

                if let Ok(expander) = read_expander_csv(3) {
                    ui.set_speedHundredsA(expander[15]);
                    ui.set_speedTensF(expander[14]);
                    ui.set_speedTensA(expander[13]);
                    ui.set_speedTensG(expander[12]);
                    ui.set_speedTensB(expander[11]);
                    ui.set_speedOnesF(expander[10]);
                    ui.set_speedOnesA(expander[9]);
                    ui.set_speedOnesB(expander[8]);
                    ui.set_speedHundredsB(expander[7]);
                    ui.set_speedTensD(expander[6]);
                    ui.set_speedTensE(expander[5]);
                    ui.set_speedOnesD(expander[4]);
                    ui.set_speedTensC(expander[3]);
                    ui.set_speedOnesE(expander[2]);
                    ui.set_speedOnesC(expander[1]);
                    ui.set_speedOnesG(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(4) {
                    ui.set_readyIndicator(expander[15]);
                    ui.set_unused1(expander[14]);
                    ui.set_unused2(expander[13]);
                    ui.set_driveModeR(expander[12]);
                    ui.set_driveModeD1(expander[11]);
                    ui.set_driveModeD2(expander[10]);
                    ui.set_driveModeD3(expander[9]);
                    ui.set_driveModeP(expander[8]);
                    ui.set_unused3(expander[7]);
                    ui.set_unused4(expander[6]);
                    ui.set_unused5(expander[5]);
                    ui.set_unused6(expander[4]);
                    ui.set_unused7(expander[3]);
                    ui.set_alarmRed(expander[2]);
                    ui.set_unused9(expander[1]);
                    ui.set_unused10(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(5) {
                    ui.set_clockHourTensC(expander[15]);
                    ui.set_clockHourTensD(expander[14]);
                    ui.set_clockHourTensE(expander[13]);
                    ui.set_clockHourTensG(expander[12]);
                    ui.set_clockMinusIndicator(expander[11]);
                    ui.set_clockHourTensF(expander[10]);
                    ui.set_clockHourTensA(expander[9]);
                    ui.set_clockHourTensB(expander[8]);
                    ui.set_clockColonIndicator(expander[7]);
                    ui.set_clockHourOnesA(expander[6]);
                    ui.set_clockHourOnesB(expander[5]);
                    ui.set_clockHourOnesG(expander[4]);
                    ui.set_clockHourOnesC(expander[3]);
                    ui.set_clockHourOnesD(expander[2]);
                    ui.set_clockHourOnesE(expander[1]);
                    ui.set_clockHourOnesF(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(6) {
                    ui.set_tripOnesDF(expander[15]);
                    ui.set_tripOnesD(expander[14]);
                    ui.set_tripOnesE(expander[13]);
                    ui.set_tripOnesG(expander[12]);
                    ui.set_tripOnesF(expander[11]);
                    ui.set_tripOnesA(expander[10]);
                    ui.set_tripOnesB(expander[9]);
                    ui.set_tripOnesC(expander[8]);
                    ui.set_tripKmIndicator(expander[7]);
                    ui.set_tripFractionA(expander[6]);
                    ui.set_tripFractionB(expander[5]);
                    ui.set_tripFractionG(expander[4]);
                    ui.set_tripFractionC(expander[3]);
                    ui.set_tripFractionD(expander[2]);
                    ui.set_tripFractionE(expander[1]);
                    ui.set_tripFractionF(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(7) {
                    ui.set_clockTemperatureIndicator(expander[15]);
                    ui.set_powerMinus20(expander[14]);
                    ui.set_powerMinus30(expander[13]);
                    ui.set_battery1_10(expander[12]);
                    ui.set_battery1_20(expander[11]);
                    ui.set_taillights(expander[10]);
                    ui.set_headlights(expander[9]);
                    ui.set_farlights(expander[8]);
                    ui.set_battery1_30(expander[7]);
                    ui.set_battery1_40(expander[6]);
                    ui.set_battery1_50(expander[5]);
                    ui.set_battery1_60(expander[4]);
                    ui.set_battery1_70(expander[3]);
                    ui.set_battery1_80(expander[2]);
                    ui.set_battery1_90(expander[1]);
                    ui.set_battery1_100(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(8) {
                    ui.set_serviceAlarm(expander[15]);
                    ui.set_gsmIndicator(expander[14]);
                    ui.set_battery2_50(expander[13]);
                    ui.set_battery2_40(expander[12]);
                    ui.set_kmh(expander[11]);
                    ui.set_battery2_30(expander[10]);
                    ui.set_battery2_20(expander[9]);
                    ui.set_battery2_10(expander[8]);
                    ui.set_range(expander[7]);
                    ui.set_powerMinus10(expander[6]);
                    ui.set_trip(expander[5]);
                    ui.set_powerPlus10(expander[4]);
                    ui.set_odo(expander[3]);
                    ui.set_powerPlus20(expander[2]);
                    ui.set_powerPlus30(expander[1]);
                    ui.set_powerPlus40(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(9) {
                    ui.set_tripTenthousandsE(expander[15]);
                    ui.set_tripTenthousandsG(expander[14]);
                    ui.set_tripTenthousandsF(expander[13]);
                    ui.set_tripTenthousandsA(expander[12]);
                    ui.set_tripHundredthousandsBAlt(expander[11]);
                    ui.set_tripHundredthousandsAUst(expander[10]);
                    ui.set_powerPlus60(expander[9]);
                    ui.set_powerPlus50(expander[8]);
                    ui.set_tripThousandsD(expander[7]);
                    ui.set_tripThousandsA(expander[6]);
                    ui.set_tripThousandsE(expander[5]);
                    ui.set_tripThousandsG(expander[4]);
                    ui.set_tripThousandsF(expander[3]);
                    ui.set_tripTenthousandsB(expander[2]);
                    ui.set_tripTenthousandsC(expander[1]);
                    ui.set_tripTenthousandsD(expander[0]);
                }

                if let Ok(expander) = read_expander_csv(10) {
                    ui.set_tripHundredsB(expander[15]);
                    ui.set_tripHundredsG(expander[14]);
                    ui.set_tripHundredsC(expander[13]);
                    ui.set_tripHundredsD(expander[12]);
                    ui.set_tripHundredsE(expander[11]);
                    ui.set_tripHundredsF(expander[10]);
                    ui.set_tripThousandsB(expander[9]);
                    ui.set_tripThousandsC(expander[8]);
                    ui.set_tripHundredsA(expander[7]);
                    ui.set_tripTensG(expander[6]);
                    ui.set_tripTensB(expander[5]);
                    ui.set_tripTensC(expander[4]);
                    ui.set_tripTensD(expander[3]);
                    ui.set_tripTensE(expander[2]);
                    ui.set_tripTensF(expander[1]);
                    ui.set_tripTensA(expander[0]);
                }
            },
        );

        ui.run()
    });

    let _ = tokio::join!(reader_task, writer_task, ui_task);
    Ok(())
}

/// Send a standard CAN frame to the serial port.
async fn send_frame(writer: &mut (impl AsyncWriteExt + Unpin), frame: CANFrame) {
    let frame_bytes = frame.to_bytes();
    if let Err(e) = writer.write_all(&frame_bytes).await {
        eprintln!("Error writing to serial port: {:?}", e);
    } else {
        println!("Sent CAN frame: {:?}", frame);
    }
}

/// Send an extended CAN frame using our custom framing.
/// This ensures the start (0xAA) and end (0x55) markers are present,
/// so that the USB‑CAN Analyzer picks up the extended frame.
async fn send_extended_frame(writer: &mut (impl AsyncWriteExt + Unpin), frame: ExtendedCANFrame) {
    let frame_bytes = frame.to_bytes();
    if let Err(e) = writer.write_all(&frame_bytes).await {
        eprintln!("Error writing extended frame: {:?}", e);
    } else {
        println!("Sent Extended CAN frame: {:?}", frame);
    }
}

pub async fn handle_expander(id: u32, data: &[u8]) {
    // Ensure we have exactly 2 bytes of data.
    eprintln!("Handling id: {}, with data: {:?}", id, data);
    if data.len() < 2 {
        eprintln!(
            "Expected at least 2 bytes of data for expander id {}, but got {}",
            id,
            data.len()
        );
        return;
    }

    // Validate the id is within the range 100 to 109.
    if id < 100 || id > 109 {
        eprintln!(
            "Invalid expander id: {}. Expected value between 100 and 109.",
            id
        );
        return;
    }

    // Map CAN frame id to expander number: 100 -> 1, 101 -> 2, ..., 109 -> 10.
    let expander_num = id - 99;
    let file_path = format!("expanders/expander{}.csv", expander_num);

    // Convert the first two bytes into a vector of bit strings (MSB-first order).
    let mut bits = Vec::with_capacity(16);
    for byte in &data[0..2] {
        // Iterate from bit 7 down to bit 0.
        for i in (0..8).rev() {
            let bit = (byte >> i) & 1;
            bits.push(bit.to_string());
        }
    }

    // Create a CSV row by joining the bit strings with commas.
    let row = bits.join(",");

    // Ensure the directory exists.
    if let Some(parent) = Path::new(&file_path).parent() {
        if let Err(e) = create_dir_all(parent).await {
            eprintln!("Error creating directory {:?}: {}", parent, e);
            return;
        }
    }

    // Open the CSV file in write mode with truncation.
    let mut file = match OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true) // This will overwrite the file each time.
        .open(&file_path)
        .await
    {
        Ok(f) => f,
        Err(e) => {
            eprintln!("Could not open file {}: {}", file_path, e);
            return;
        }
    };

    // Write the CSV row, ending with a newline.
    if let Err(e) = file.write_all(row.as_bytes()).await {
        eprintln!("Error writing to file {}: {}", file_path, e);
    }
    if let Err(e) = file.write_all(b"\n").await {
        eprintln!("Error writing newline to file {}: {}", file_path, e);
    }
}

pub fn read_expander_csv(expander_num: u32) -> Result<[bool; 16], Box<dyn std::error::Error>> {
    let file_path = format!("expanders/expander{}.csv", expander_num);

    // Open the file.
    let mut file =
        File::open(&file_path).map_err(|e| format!("Failed to open {}: {}", file_path, e))?;

    // Read the file content into a string.
    let mut contents = String::new();
    file.read_to_string(&mut contents)
        .map_err(|e| format!("Failed to read {}: {}", file_path, e))?;

    // Get the first non-empty line.
    let line = contents
        .lines()
        .find(|line| !line.trim().is_empty())
        .ok_or("CSV file is empty or has no valid data")?;

    // Split the line by commas, trim whitespace, and parse each value as u8,
    // converting each value to a bool.
    let bool_values: Result<Vec<bool>, Box<dyn std::error::Error>> = line
        .split(',')
        .map(|s| {
            let trimmed = s.trim();
            let value: u8 = trimmed
                .parse()
                .map_err(|_| format!("Failed to parse '{}' as u8", trimmed))?;
            match value {
                0 => Ok(false),
                1 => Ok(true),
                _ => Err(format!("Invalid value '{}': expected 0 or 1", value).into()),
            }
        })
        .collect();

    let bool_values = bool_values?;

    // Ensure that we got exactly 16 values.
    if bool_values.len() != 16 {
        return Err(format!(
            "Expected 16 values in {}, but got {}",
            file_path,
            bool_values.len()
        )
        .into());
    }

    // Convert the Vec<bool> into an array of 16 elements.
    let array: [bool; 16] = bool_values
        .try_into()
        .map_err(|_| "Conversion to array failed")?;

    Ok(array)
}
