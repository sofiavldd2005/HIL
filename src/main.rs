
use serialport::SerialPortType;
use std::default;
use std::io::Write;
use std::time::{Duration,Instant};
use std::f32::consts::PI;


#[derive(Default)] //to initalise to default (basicamente tds os floats a 0.0)
struct vn_simul_data{  // to be sent via serial port

	// char array : [0] a [129]
	// first 8 bytes are headers corresponding to the groups
	yaw: f32,
    pitch: f32,
    roll: f32,
    uncomp_accel: [f32; 3],
    uncomp_gyro: [f32; 3],
    temp: f32,
    press: f32,
    mag: [f32; 3],
    accel: [f32; 3],
    gyro: [f32; 3],
    quat: [f32; 4],
    lin_accel: [f32; 3],
    accel_ned: [f32; 3],
}

fn find_ftdi_port() -> Option<String> {
    let ports = serialport::available_ports().expect("No ports found!");
    
    for p in ports {
        // Search for names containing ttyUSB or ttyACM
        if p.port_name.contains("ttyACM"){
            println!("Found potential port: {}", p.port_name);
            return Some(p.port_name);
        }
    }
    None
}

fn calculate_crc (data: &[u8])-> u16{
    let mut crc: u16 = 0;

    for &byte in &data[1..128]{
        crc = (crc >> 8) | (crc << 8);
        crc ^= byte as u16;
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00FF) << 5; 
    }
    crc
}
fn main() {
    println!("Hello, world!");

    let port_name = find_ftdi_port().expect("USB Serial port not found. Is the ftdi plugged in?"); //instead of hardcoding the port as before
    let baud_rate: u32 = 921600;
    let frequency : f32 = 114.0; //Hz
    let interval = Duration::from_secs_f32((1.0/frequency));


    ///Open serial port
    
    let mut port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(1000))
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .open()
        .expect("Failed to open port");

    let mut sim = vn_simul_data::default();
    let mut loop_timer = Instant::now();


    println!("Starting Rust HIL Simulation at {}Hz...", frequency);

    loop {
        let start = Instant::now();

        sim.yaw = (sim.yaw +1.0);
        sim.pitch = (sim.pitch +1.0);
        sim.roll = (sim.roll+1.0);
        sim.uncomp_accel = [1.0,2.0,3.0];
        sim.uncomp_gyro = [1.0,2.0,3.0];
        sim.temp= 20.0;
        sim.press = 100.000;
        sim.mag= [1.0,2.0,3.0];
        sim.gyro = [1.0,2.0,3.0];
        sim.accel = [1.0,2.0,3.0];
        sim.lin_accel = [1.0,2.0,3.0];
        sim.accel_ned = [1.0,2.0,3.0];

        let mut p: Vec<u8> = vec![0; 130]; // o nosso "char" (uint8 actually) array
        
        //Initalize headers
        p[0] =  0xFA;
	    p[1] =  0x15;
	    p[2] =  0x08;
	    p[3] =  0x02;
	    p[4] =  0x30;
	    p[5] =  0x07;
	    p[6] =  0xC4;
	    p[7] =  0x00;

        // Euler Angles [8-19]
        p[8..12].copy_from_slice(&sim.yaw.to_le_bytes());
        p[12..16].copy_from_slice(&sim.pitch.to_le_bytes());
        p[16..20].copy_from_slice(&sim.roll.to_le_bytes());

        // Uncomp Accel [20-31]
        p[20..24].copy_from_slice(&sim.uncomp_accel[0].to_le_bytes());
        p[24..28].copy_from_slice(&sim.uncomp_accel[1].to_le_bytes());
        p[28..32].copy_from_slice(&sim.uncomp_accel[2].to_le_bytes());

        // Uncomp Gyro [32-43]
        p[32..36].copy_from_slice(&sim.uncomp_gyro[0].to_le_bytes());
        p[36..40].copy_from_slice(&sim.uncomp_gyro[1].to_le_bytes());
        p[40..44].copy_from_slice(&sim.uncomp_gyro[2].to_le_bytes());

        // Temp & Pressure [44-51]
        p[44..48].copy_from_slice(&sim.temp.to_le_bytes());
        p[48..52].copy_from_slice(&sim.press.to_le_bytes());

        // Mag [52-63]
        p[52..56].copy_from_slice(&sim.mag[0].to_le_bytes());
        p[56..60].copy_from_slice(&sim.mag[1].to_le_bytes());
        p[60..64].copy_from_slice(&sim.mag[2].to_le_bytes());

        // Accel [64-75]
        p[64..68].copy_from_slice(&sim.accel[0].to_le_bytes());
        p[68..72].copy_from_slice(&sim.accel[1].to_le_bytes());
        p[72..76].copy_from_slice(&sim.accel[2].to_le_bytes());

        // Gyro [76-87]
        p[76..80].copy_from_slice(&sim.gyro[0].to_le_bytes());
        p[80..84].copy_from_slice(&sim.gyro[1].to_le_bytes());
        p[84..88].copy_from_slice(&sim.gyro[2].to_le_bytes());

        // Quat [88-103]
        p[88..92].copy_from_slice(&sim.quat[0].to_le_bytes());
        p[92..96].copy_from_slice(&sim.quat[1].to_le_bytes());
        p[96..100].copy_from_slice(&sim.quat[2].to_le_bytes());
        p[100..104].copy_from_slice(&sim.quat[3].to_le_bytes());

        // Lin Accel [104-115]
        p[104..108].copy_from_slice(&sim.lin_accel[0].to_le_bytes());
        p[108..112].copy_from_slice(&sim.lin_accel[1].to_le_bytes());
        p[112..116].copy_from_slice(&sim.lin_accel[2].to_le_bytes());

        // Accel NED [116-127]
        p[116..120].copy_from_slice(&sim.accel_ned[0].to_le_bytes());
        p[120..124].copy_from_slice(&sim.accel_ned[1].to_le_bytes());
        p[124..128].copy_from_slice(&sim.accel_ned[2].to_le_bytes());

         //crc [128-129];
        let crc = calculate_crc(&p);
       p[128..130].copy_from_slice(&crc.to_le_bytes());

        //p[128] = 0xCC; // Dummy CRC
        //p[129] = 0xAA;
        //sendout

        port.write_all(&p).expect("Failed to write to port");
        port.flush().expect("Failed to flush port");


        let elapsed = start.elapsed();
        if elapsed < interval {
            std::thread::sleep(interval - elapsed);
        }

    }

}

