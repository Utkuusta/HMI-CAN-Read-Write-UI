use canparse::pgn::{ParseMessage, PgnLibrary, SpnDefinition};
use dbc::*;
use std::collections::HashMap;
use std::io;
use std::time::Duration;
use tokio::io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader};
use tokio_serial::SerialPortBuilderExt;

mod dbc;

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

#[tokio::main]
async fn main() -> io::Result<()> {
    let port_name = r"\\.\COM11"; // Adjust for your system.
    let baud_rate = 115200;

    // Load your DBC file (if needed).
    let _lib = PgnLibrary::from_dbc_file("./src/Riki_j1939.dbc").unwrap();

    // Open the serial port.
    let serial_port = tokio_serial::new(port_name, baud_rate)
        .timeout(Duration::from_millis(100))
        .open_native_async()?;

    // Split into reader and writer.
    let (mut reader, mut writer) = tokio::io::split(serial_port);

    // Task for reading incoming frames.
    let reader_task = tokio::spawn(async move {
        let mut buffer: Vec<u8> = Vec::new();
        let mut read_buf = [0u8; 128];
        loop {
            match reader.read(&mut read_buf).await {
                Ok(n) if n == 0 => continue,
                Ok(n) => {
                    buffer.extend_from_slice(&read_buf[..n]);
                    println!("Raw buffer in hex: {:02X?}", buffer);

                    // Process frames delimited by 0xAA ... 0x55.
                    while let Some(start) = buffer.iter().position(|&b| b == 0xAA) {
                        if let Some(end) = buffer[start..].iter().position(|&b| b == 0x55) {
                            let frame_bytes = &buffer[start + 1..start + end];
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
                            }
                            buffer.drain(..start + end + 1);
                        } else {
                            buffer.drain(..start);
                            break;
                        }
                    }
                }
                Err(e) => {
                    eprintln!("Error reading from serial port: {:?}", e);
                    break;
                }
            }
        }
    });

    // Task for handling keyboard input and sending frames.
    let writer_task = tokio::spawn(async move {
        let mut toggle_states: HashMap<u8, bool> = HashMap::new();
        let mut throttle_value: u8 = 0;
        let mut engine_speed: u16 = 0;
        let stdin = BufReader::new(tokio::io::stdin());
        let mut lines = stdin.lines();

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
                    send_frame(&mut writer, frame).await;
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
                    send_frame(&mut writer, frame).await;
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
                    send_frame(&mut writer, frame).await;
                }
                "+" => {
                    // Increase throttle_value.
                    if throttle_value < 100 {
                        throttle_value += 5;
                    }
                    let data = encode_proprietary_a(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 0x8CEF00FE,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    send_extended_frame(&mut writer, ext_frame).await;
                }
                "-" => {
                    // Decrease throttle_value.
                    if throttle_value > 0 {
                        throttle_value -= 5;
                    }
                    let data = encode_proprietary_a(throttle_value as f64);

                    // Note: The raw id (without the extended flag) is used here.
                    let ext_frame = ExtendedCANFrame {
                        id: 0x8CEF00FE,
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    send_extended_frame(&mut writer, ext_frame).await;
                }
                "++" => {
                    // Increase engine_speed and send an extended frame.
                    if engine_speed < 150 {
                        engine_speed += 5;
                    }
                    // Convert engine_speed to raw value (raw = decoded / 0.125).
                    let raw_value: u64 = ((engine_speed as f32) / 0.125).round() as u64;
                    let mut data = [0u8; 8];
                    let start_bit = 24;
                    let bit_len = 16;
                    let little_endian = true;
                    if little_endian {
                        let mut current = u64::from_le_bytes(data);
                        let mask: u64 = ((1u64 << bit_len) - 1) << start_bit;
                        current &= !mask;
                        current |= (raw_value << start_bit) & mask;
                        data = current.to_le_bytes();
                    } else {
                        let mut current = u64::from_be_bytes(data);
                        let mask: u64 = ((1u64 << bit_len) - 1) << start_bit;
                        current &= !mask;
                        current |= (raw_value << start_bit) & mask;
                        data = current.to_be_bytes();
                    }
                    // Note: The raw id (without the extended flag) is used here. 2364540158 = 0x8CF02C0E
                    let ext_frame = ExtendedCANFrame {
                        id: 0x8CF02C0E, // 0x8CF02C0E
                        dlc: 8,
                        data,
                    };
                    // IMPORTANT: Use custom framing so the analyzer can pick up the frame.
                    send_extended_frame(&mut writer, ext_frame).await;
                }
                "--" => {
                    // Decrease engine_speed and send an extended frame.
                    if engine_speed > 0 {
                        engine_speed -= 5;
                    }
                    let raw_value: u64 = ((engine_speed as f32) / 0.125).round() as u64;
                    let mut data = [0u8; 8];
                    let start_bit = 24;
                    let bit_len = 16;
                    let little_endian = true;
                    if little_endian {
                        let mut current = u64::from_le_bytes(data);
                        let mask: u64 = ((1u64 << bit_len) - 1) << start_bit;
                        current &= !mask;
                        current |= (raw_value << start_bit) & mask;
                        data = current.to_le_bytes();
                    } else {
                        let mut current = u64::from_be_bytes(data);
                        let mask: u64 = ((1u64 << bit_len) - 1) << start_bit;
                        current &= !mask;
                        current |= (raw_value << start_bit) & mask;
                        data = current.to_be_bytes();
                    }
                    let ext_frame = ExtendedCANFrame {
                        id: 2364540158, // 0x8CF02C0E
                        dlc: 8,
                        data,
                    };
                    send_extended_frame(&mut writer, ext_frame).await;
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
                    send_extended_frame(&mut writer, ext_frame).await;
                }
                _ => {
                    // Toggle states for commands 7..21.
                    if let Ok(num) = command.parse::<u8>() {
                        if (7..=21).contains(&num) {
                            let state = toggle_states.entry(num).or_insert(false);
                            *state = !*state;
                            let value = if *state { 1 } else { 0 };
                            let frame = CANFrame {
                                id: num as u16,
                                dlc: 4,
                                data: {
                                    let mut d = [0u8; 8];
                                    d[0] = value;
                                    d
                                },
                            };
                            println!("Toggled id {} to {}", num, value);
                            send_frame(&mut writer, frame).await;
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

    let _ = tokio::join!(reader_task, writer_task);
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
