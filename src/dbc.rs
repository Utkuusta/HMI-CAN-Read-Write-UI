use crate::{
    diagnostics::DM1Packet, ActiveDiagnosticTroubleCodesPDIO, ProprietaryB5States,
    SwitchedPowerOutputStates,
};

// Known CAN IDs for each message.
const ENGINE_CONTROLLER1_CAN_ID: u32 = 2364540158;
const ENGINE_CONTROLLER2_CAN_ID: u32 = 2364539902;
const HVESS_DATA1_CAN_ID: u32 = 2565866752;
const HYBRID_EV_STATUS1_CAN_ID: u32 = 2566701822;
const VEHICLE_IDENTIFICATION_CAN_ID: u32 = 2566843646;
const PROPRIETARY_A_CAN_ID: u32 = 2364473598; // New message
const PROPRIETARY_B5_CAN_ID: u32 = 2566849566;

/// Encodes the Electronic_Engine_Controller_1 message:
///   SG_ Engine_Speed : 24|16@1+ (0.125,0) "rpm"
///   SG_ Engine_Demand_Percent_Torque : 56|8@1+ (1,-125) "%"
///   SG_ Actual_Engine_Percent_Torque : 16|8@1+ (1,0) "%"  (two’s complement)
///   SG_ Actual_Engine_Percent_Torque_Fractional : 4|4@1+ (0.125,0) "%"
///   SG_ Drivers_Demand_Percent_Torque : 8|8@1+ (1,-125) "%"
///
/// The packing order is:
/// - Engine_Demand_Percent_Torque at bits 56–63,
/// - Engine_Speed at bits 24–39,
/// - Actual_Engine_Percent_Torque at bits 16–23,
/// - Drivers_Demand_Percent_Torque at bits 8–15,
/// - Actual_Engine_Percent_Torque_Fractional at bits 4–7.
pub fn encode_engine_controller1(
    engine_speed: f64,
    engine_demand_percent_torque: f64,
    actual_engine_percent_torque: f64,
    actual_engine_percent_torque_fractional: f64,
    drivers_demand_percent_torque: f64,
) -> [u8; 8] {
    let raw_engine_speed = (engine_speed / 0.125).round() as u16; // e.g. 3000/0.125 = 24000 = 0x5DC0
    let raw_engine_demand = (engine_demand_percent_torque + 125.0).round() as u8; // e.g. 50+125 = 175 = 0xAF
    let raw_actual_engine = actual_engine_percent_torque.round() as i8; // e.g. -20 becomes 0xEC (236)
    let raw_actual_engine_fractional =
        (actual_engine_percent_torque_fractional / 0.125).round() as u8; // e.g. 0.5/0.125 = 4
    let raw_drivers_demand = (drivers_demand_percent_torque + 125.0).round() as u8; // e.g. 0+125 = 125 = 0x7D

    let mut data: u64 = 0;
    data |= (raw_engine_speed as u64) << 24;
    data |= ((raw_actual_engine as u8) as u64) << 16;
    data |= (raw_drivers_demand as u64) << 8;
    data |= (raw_actual_engine_fractional as u64) << 4;
    data |= (raw_engine_demand as u64) << 56;
    data.to_le_bytes()
}

/// Encodes the Electronic_Engine_Controller_2 message:
///   SG_ Engine_Percent_Load_At_Current_Speed : 16|8@1+ (1,0) "%"  
///   SG_ Accelerator_Pedal_1_Position : 8|8@1+ (0.4,0) "%"  
///   SG_ Road_Speed_Limit_Status : 4|2@1+ (1,0) ""
///
/// The packing order is:
/// - Engine_Percent_Load at bits 16–23,
/// - Accelerator_Pedal_1_Position at bits 8–15,
/// - Road_Speed_Limit_Status at bits 4–5.
pub fn encode_engine_controller2(
    engine_percent_load: f64, // scale=1, offset=0.
    accelerator_pedal: f64,   // scale=0.4, offset=0.
    road_speed_limit: u8,     // 2 bits.
) -> [u8; 8] {
    let raw_load = engine_percent_load.round() as u8;
    let raw_accel = (accelerator_pedal / 0.4).round() as u8;
    let raw_road = road_speed_limit & 0x03; // ensure only 2 bits are used

    let mut data: u64 = 0;
    data |= (raw_load as u64) << 16;
    data |= (raw_accel as u64) << 8;
    data |= (raw_road as u64) << 4;
    data.to_le_bytes()
}

/// Encodes the High_Voltage_Energy_Storage_System_Data_1 message:
///   SG_ HVESS_Available_Discharge_Power : 0|16@1+ (0.05,0) "kW"
///   SG_ HVESS_Available_Charge_Power : 16|16@1+ (0.05,0) "kW"
///   SG_ HVESS_Voltage_Level : 32|16@1+ (0.05,0) "V"
///   SG_ HVESS_Current : 48|16@1- (0.05,0) "A"  (signed)
///
/// Each signal’s raw value is computed as physical value divided by 0.05.
/// They are placed in order: discharge (bits 0–15), charge (16–31),
/// voltage (32–47) and current (48–63).
pub fn encode_hvess_data1(
    available_discharge: f64,
    available_charge: f64,
    voltage_level: f64,
    current: f64,
) -> [u8; 8] {
    let raw_discharge = (available_discharge / 0.05).round() as u16;
    let raw_charge = (available_charge / 0.05).round() as u16;
    let raw_voltage = (voltage_level / 0.05).round() as u16;
    let raw_current = (current / 0.05).round() as i16; // signed

    let mut data: u64 = 0;
    data |= raw_discharge as u64; // bits 0–15
    data |= (raw_charge as u64) << 16; // bits 16–31
    data |= (raw_voltage as u64) << 32; // bits 32–47
    data |= ((raw_current as u16) as u64) << 48; // bits 48–63 (two's complement)
    data.to_le_bytes()
}

/// Encodes the Hybrid_or_EV_System_Status_1 message:
///   SG_ Regenerative_Braking_Indicator : 20|2@1+ (1,0) ""
///
/// The value (0–3) is placed at bits 20–21.
pub fn encode_hybrid_ev_status1(regen_brake_indicator: u8) -> [u8; 8] {
    let raw_regen = regen_brake_indicator & 0x03;
    let mut data: u64 = 0;
    data |= (raw_regen as u64) << 20;
    data.to_le_bytes()
}

/// Encodes the Vehicle_Identification message:
///   SG_ VIN : 0|64@4- (1,0) ""
///
/// The 64-bit VIN is simply represented as 8 bytes.
pub fn encode_vehicle_identification(vin: u64) -> [u8; 8] {
    vin.to_le_bytes()
}

/// Encodes the Proprietary_B4 message:
///   SG_ Accelerator_Pedal_1_Position : 8|8@1+ (0.4,0) [0|100] "%"
///
/// The signal is placed at bits 8–15 of the 8-byte frame.
/// Raw value is computed as physical_value / 0.4.
pub fn encode_proprietary_b4(accelerator_pedal_position: f64) -> [u8; 8] {
    let raw = (accelerator_pedal_position / 0.4).round() as u8;
    let mut data: u64 = 0;
    data |= (raw as u64) << 8;
    data.to_le_bytes()
}

/// Encodes the Proprietary_B5 message:
///   SG_ Kickstand_Switch   : 0|2@1+ (1,0) [0|1]
///   SG_ Brake_Switch_Front : 16|2@1+ (1,0) [0|1]
///   SG_ Brake_Switch_Rear  : 24|2@1+ (1,0) [0|1]
///
/// Each signal uses 2 bits (even if the value is only 0 or 1), little-endian, unsigned.
pub fn encode_proprietary_b5(proprietary_b5_states: ProprietaryB5States) -> [u8; 8] {
    let mut data: u64 = 0;
    data |= ((proprietary_b5_states.kickstand_switch & 0x03) as u64) << 0; // bits 0–1
    data |= ((proprietary_b5_states.brake_switch_front & 0x03) as u64) << 16; // bits 16–17
    data |= ((proprietary_b5_states.brake_switch_rear & 0x03) as u64) << 24; // bits 24–25
    data.to_le_bytes()
}

/// Encodes the Switched_Power_Output_Status message:
///   SG_ Output_0_State             : 0|2@1+
///   SG_ Output_1_State             : 2|2@1+
///   SG_ Output_2_State             : 4|2@1+
///   SG_ Output_3_State             : 6|2@1+
///   SG_ Output_4_State             : 8|2@1+
///   SG_ Output_5_State             : 10|2@1+
///   SG_ Output_6_State             : 12|2@1+
///   SG_ Output_7_State             : 14|2@1+
///   SG_ Output_8_State             : 16|2@1+
///   SG_ Output_9_State             : 18|2@1+
///   SG_ Output_10_State            : 20|2@1+
///   SG_ Output_HighCurrent_0_State: 48|2@1+
///   SG_ Output_HighCurrent_1_State: 50|2@1+
///
/// Each signal is 2 bits, unsigned, placed at its specified bit offset.
pub fn encode_switched_power_output_status(
    switched_power_output_states: SwitchedPowerOutputStates,
) -> [u8; 8] {
    let mut data: u64 = 0;

    data |= (switched_power_output_states.output_0 as u64) << 0;
    data |= (switched_power_output_states.output_1 as u64) << 2;
    data |= (switched_power_output_states.output_2 as u64) << 4;
    data |= (switched_power_output_states.output_3 as u64) << 6;
    data |= (switched_power_output_states.output_4 as u64) << 8;
    data |= (switched_power_output_states.output_5 as u64) << 10;
    data |= (switched_power_output_states.output_6 as u64) << 12;
    data |= (switched_power_output_states.output_7 as u64) << 14;
    data |= (switched_power_output_states.output_8 as u64) << 16;
    data |= (switched_power_output_states.output_9 as u64) << 18;
    data |= (switched_power_output_states.output_10 as u64) << 20;

    data |= (switched_power_output_states.high_current_0 as u64) << 48;
    data |= (switched_power_output_states.high_current_1 as u64) << 50;

    data.to_le_bytes()
}

pub fn encode_active_diagnostic_trouble_codes_pdio(
    dtc: ActiveDiagnosticTroubleCodesPDIO,
) -> [u8; 8] {
    let packet = DM1Packet::new()
        .with_protect_lamp(dtc.protect_lamp)
        .with_amber_warning(dtc.amber_warning)
        .with_red_stop(dtc.red_stop)
        .with_malfunction(dtc.malfunction)
        .with_protect_lamp_flash(dtc.protect_lamp_flash)
        .with_amber_warning_flash(dtc.amber_warning_flash)
        .with_red_stop_flash(dtc.red_stop_flash)
        .with_malfunction_flash(dtc.malfunction_flash)
        .with_spn(dtc.source_spn & 0x7FFFF)
        .with_fmi(dtc.failure_mode)
        .with_occurence_count(dtc.occurrence_count & 0x7F)
        .with_spn_conversion_method_is_old(dtc.conv_method_old)
        .with_reserved(dtc.reserved);

    packet.into_bits().to_le_bytes()
}

/*
fn main() {
    // Electronic_Engine_Controller_1 example:
    let engine_speed = 3000.0; // rpm
    let engine_demand_percent_torque = 50.0; // %
    let actual_engine_percent_torque = -20.0; // % (raw two's complement: 236)
    let actual_engine_percent_torque_fractional = 0.5; // %
    let drivers_demand_percent_torque = 0.0; // %
    let frame_data1 = encode_engine_controller1(
        engine_speed,
        engine_demand_percent_torque,
        actual_engine_percent_torque,
        actual_engine_percent_torque_fractional,
        drivers_demand_percent_torque,
    );
    let frame_data_str1 = frame_data1
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "Engine Controller 1 (ID: {:08X})\n  Physical values: engine_speed: {:.1} rpm, engine_demand: {:.1}%, actual_engine: {:.1}%, actual_engine_fractional: {:.2}%, drivers_demand: {:.1}%\n  Data: {}\n",
        ENGINE_CONTROLLER1_CAN_ID,
        engine_speed,
        engine_demand_percent_torque,
        actual_engine_percent_torque,
        actual_engine_percent_torque_fractional,
        drivers_demand_percent_torque,
        frame_data_str1
    );

    // Electronic_Engine_Controller_2 example:
    let engine_percent_load = 100.0; // %
    let accelerator_pedal = 20.0; // physical; raw = 20/0.4 = 50
    let road_speed_limit = 2; // value between 0 and 3
    let frame_data2 =
        encode_engine_controller2(engine_percent_load, accelerator_pedal, road_speed_limit);
    let frame_data_str2 = frame_data2
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "Engine Controller 2 (ID: {:08X})\n  Physical values: engine_percent_load: {:.1}%, accelerator_pedal: {:.1}, road_speed_limit: {}\n  Data: {}\n",
        ENGINE_CONTROLLER2_CAN_ID,
        engine_percent_load,
        accelerator_pedal,
        road_speed_limit,
        frame_data_str2
    );

    // High_Voltage_Energy_Storage_System_Data_1 example:
    let available_discharge = 100.0; // kW
    let available_charge = 80.0; // kW
    let voltage_level = 400.0; // V
    let current = -50.0; // A (negative value)
    let frame_data3 = encode_hvess_data1(
        available_discharge,
        available_charge,
        voltage_level,
        current,
    );
    let frame_data_str3 = frame_data3
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "HVESS Data 1 (ID: {:08X})\n  Physical values: available_discharge: {:.1} kW, available_charge: {:.1} kW, voltage_level: {:.1} V, current: {:.1} A\n  Data: {}\n",
        HVESS_DATA1_CAN_ID,
        available_discharge,
        available_charge,
        voltage_level,
        current,
        frame_data_str3
    );

    // Hybrid_or_EV_System_Status_1 example:
    let regen_brake_indicator = 1; // value between 0 and 3
    let frame_data4 = encode_hybrid_ev_status1(regen_brake_indicator);
    let frame_data_str4 = frame_data4
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "Hybrid/EV Status 1 (ID: {:08X})\n  Physical value: regen_brake_indicator: {}\n  Data: {}\n",
        HYBRID_EV_STATUS1_CAN_ID,
        regen_brake_indicator,
        frame_data_str4
    );

    // Vehicle_Identification example:
    let vin: u64 = 0x1A2B3C4D5E6F7081; // dummy VIN
    let frame_data5 = encode_vehicle_identification(vin);
    let frame_data_str5 = frame_data5
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "Vehicle Identification (ID: {:08X})\n  Physical VIN: {:016X}\n  Data: {}\n",
        VEHICLE_IDENTIFICATION_CAN_ID, vin, frame_data_str5
    );

    // Proprietary_A example:
    let accelerator_pedal_position = 40.0; // physical value; raw = 40/0.4 = 100 (0x64)
    let frame_data6 = encode_proprietary_a(accelerator_pedal_position);
    let frame_data_str6 = frame_data6
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ");
    println!(
        "Proprietary_A (ID: {:08X})\n  Physical value: Accelerator_Pedal_1_Position: {:.1}%\n  Data: {}\n",
        PROPRIETARY_A_CAN_ID,
        accelerator_pedal_position,
        frame_data_str6
    );
}

*/
