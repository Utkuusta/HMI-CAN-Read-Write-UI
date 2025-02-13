use serialport::SerialPort;
use std::fs;
use std::io::{self, Read, Write};
use std::time::Duration;

#[derive(Debug)]
pub struct CANFrame {
    id: u16,
    dlc: u8,
    data: [u8; 8],
}

impl CANFrame {
    fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 4 {
            return None; // Not enough data for a complete frame
        }

        // Extract CAN ID (11-bit ID from bytes[1] and part of bytes[2])
        let id_hex1 = bytes[2] & 0x07;
        let read_id = ((id_hex1 as u16) << 8) | bytes[1] as u16;
        println!("Read ID: {}", read_id);

        // Extract Data Length Code (DLC)
        let dlc = bytes[0] & 0x0F;
        if dlc > 8 {
            return None; // Invalid DLC
        }
        println!("Read DLC: {}", &dlc);

        // Extract Data Bytes (starting from the fifth byte)
        let mut data = [0u8; 8];

        for n in 0..dlc {
            if 3 + n as usize >= bytes.len() {
                return None; // Not enough bytes for the data
            }
            data[n as usize] = bytes[3 + n as usize];
        }

        println!("Extracted data bytes: {:?}", data);

        Some(CANFrame {
            id: read_id,
            dlc,
            data,
        })
    }
}

fn main() {
    let port_name = "COM25"; // Replace with your actual serial port
    let baud_rate = 115200; // Set baud rate for CAN communication

    // Open the serial port
    let mut port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(100))
        .open()
        .expect("Failed to open serial port");

    let mut buffer: Vec<u8> = Vec::new();
    let mut read_buf = [0u8; 128]; // Temporary buffer for serial port reads

    loop {
        match port.read(&mut read_buf) {
            Ok(n) => {
                if n == 0 {
                    continue; // No data read
                }

                // Append the read bytes to the buffer
                buffer.extend_from_slice(&read_buf[..n]);

                // Debug: Print the raw buffer in hex and bits
                println!("Raw buffer in hex: {:02X?}", buffer);
                let raw_bits: String = buffer
                    .iter()
                    .map(|b| format!("{:08b}", b))
                    .collect::<Vec<_>>()
                    .join(" ");
                println!("Raw buffer in bits: {}\n", raw_bits);

                // Look for a complete CAN frame, starting with 0xAA and ending with 0x55
                while let Some(start) = buffer.iter().position(|&b| b == 0xAA) {
                    if let Some(end) = buffer[start..].iter().position(|&b| b == 0x55) {
                        let frame_bytes = &buffer[start + 1..start + end];
                        if frame_bytes.len() < 4 {
                            buffer.drain(..start + end + 1); // Remove incomplete frames
                            continue;
                        }
                        if let Some(frame) = CANFrame::from_bytes(frame_bytes) {
                            println!(
                                "Interpreted CAN Frame: ID: {}, DLC: {}, Data: {:02X?}",
                                frame.id,
                                frame.dlc,
                                &frame.data[..frame.dlc as usize]
                            );
                            handle_message(frame);
                        }
                        buffer.drain(..start + end + 1); // Remove processed frame from buffer
                    } else {
                        // If there's no end marker, remove everything up to the start marker and wait for more data
                        buffer.drain(..start);
                        break;
                    }
                }

                // Remove any trailing bytes after the last processed frame
                if let Some(last_aa) = buffer.iter().rposition(|&b| b == 0xAA) {
                    if last_aa != 0 {
                        buffer.drain(..last_aa); // Keep only the last unprocessed part starting from the last `0xAA`
                    }
                } else {
                    buffer.clear(); // If no start marker remains, clear the buffer
                }
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => {
                // Timeout occurred, no data read within the specified duration
                continue;
            }
            Err(e) => {
                eprintln!("Error reading from serial port: {:?}", e);
                break;
            }
        }
    }
}

fn handle_message(frame: CANFrame) {
    // Handle each specific CAN frame ID with a match statement.
    match frame.id {
        0 if frame.dlc >= 2 => {
            update_speed(u16::from_be_bytes([frame.data[0], frame.data[1]])).unwrap()
        }
        1..=8 => update_single_led_status((frame.id - 1) as usize, frame.data[0]).unwrap(),
        10 if frame.dlc >= 2 => {
            let data: u16 = (frame.data[0] as u16) << 8 | frame.data[1] as u16;
            update_all_led_status(data).unwrap();
        }
        _ => {}
    }
}

fn update_single_led_status(line_index: usize, data: u8) -> io::Result<()> {
    let value = if data == 1 { "true" } else { "false" };

    println!("Updating line index: {}, with value: {}", line_index, value);

    // Read the current contents of the file.
    let path = "C:\\Users\\Lenovo\\Desktop\\RustProjects\\slint-ui-hmi-outputs\\led_status.txt";
    println!("Reading file from path: {}", path);
    let mut lines: Vec<String> = fs::read_to_string(path)?
        .lines()
        .map(String::from)
        .collect();
    println!("Current file contents: {:?}", lines);

    // Ensure the lines vector has exactly 9 elements.
    while lines.len() < 10 {
        println!("Adding missing line to ensure 9 elements.");
        lines.push("false".to_string());
    }

    // Update the corresponding line with the new value.
    if line_index < lines.len() {
        println!("Updating line {} with value {}", line_index, value);
        lines[line_index] = value.to_string();
    }

    // Write the updated contents back to the file.
    let contents = lines.join("\n");
    println!("Writing updated contents to file: {:?}", contents);
    let mut file = fs::File::create(path)?;
    file.write_all(contents.as_bytes())
}

fn update_all_led_status(data: u16) -> io::Result<()> {
    // Read the current contents of the file.
    let path = "C:\\Users\\Lenovo\\Desktop\\RustProjects\\slint-ui-hmi-outputs\\led_status.txt";
    let mut lines: Vec<String> = fs::read_to_string(path)?
        .lines()
        .map(String::from)
        .collect();

    // Ensure the lines vector has exactly 9 elements.
    while lines.len() < 10 {
        lines.push("false".to_string());
    }

    // Iterate through the first 10 bits of the `u16` value.
    for i in 0..10 {
        let bit_value = (data >> i) & 1;
        let value = if bit_value == 1 { "true" } else { "false" };

        // Update the corresponding line with the new value.
        lines[i] = value.to_string();
    }

    // Write the updated contents back to the file.
    let contents = lines.join("\n");
    let mut file = fs::File::create(path)?;
    file.write_all(contents.as_bytes())
}

fn update_speed(speed: u16) -> io::Result<()> {
    // Define the path to the new file for the speed value.
    let path = "C:\\Users\\Lenovo\\Desktop\\RustProjects\\slint-ui-hmi-outputs\\speed.txt";

    // Convert the speed value to a string to be written.
    let speed_str = speed.to_string();

    // Write the speed value to the file, overwriting any existing content.
    let mut file = fs::File::create(path)?;
    file.write_all(speed_str.as_bytes())
}
