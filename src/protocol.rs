use serde::Deserialize;
#[derive(Debug, Deserialize)] // deserialize to directly map from csv to struct
pub struct VnSimulData {
    // to be sent via serial port

    // char array : [0] a [129]
    // first 8 bytes are headers corresponding to the groups
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub uncomp_accel_x: f32,
    pub uncomp_accel_y: f32,
    pub uncomp_accel_z: f32,
    pub uncomp_gyro_x: f32,   
    pub uncomp_gyro_y: f32,
    pub uncomp_gyro_z: f32,
    pub temp: f32,
    pub press: f32,
    pub mag_x: f32,
    pub mag_y: f32,
    pub mag_z: f32,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub quat_x: f32,
    pub quat_y: f32,
    pub quat_z: f32,
    pub quat_w: f32,
    pub lin_accel_x: f32,
    pub lin_accel_y: f32,
    pub lin_accel_z: f32,
    pub accel_n: f32,
    pub accel_e: f32,
    pub accel_d: f32,
}

impl VnSimulData {
    pub fn packet_to_vn (&self)->  Vec<u8>{ // to initialize the binary array to be sent via the serial port
        
        let mut packet : Vec<u8> = vec![0; 130]; //initialize an array to zeros with the vec! macro
        //Initalize headers -- see Vn Control center
        packet[0] =  0xFA; //sync bit
	    packet[1] =  0x15;
	    packet[2] =  0x08;
	    packet[3] =  0x02;
	    packet[4] =  0x30;
	    packet[5] =  0x07;
	    packet[6] =  0xC4;
	    packet[7] =  0x00;

        // Payload 
        
        // Euler Angles [8-19]
        packet[8..12].copy_from_slice(&self.yaw.to_le_bytes());
        packet[12..16].copy_from_slice(&self.pitch.to_le_bytes());
        packet[16..20].copy_from_slice(&self.roll.to_le_bytes());

        // Uncomp Accel [20-31]
        packet[20..24].copy_from_slice(&self.uncomp_accel_x.to_le_bytes());
        packet[24..28].copy_from_slice(&self.uncomp_accel_y.to_le_bytes());
        packet[28..32].copy_from_slice(&self.uncomp_accel_z.to_le_bytes());

        // Uncomp Gyro [32-43]
        packet[32..36].copy_from_slice(&self.uncomp_gyro_x.to_le_bytes());
        packet[36..40].copy_from_slice(&self.uncomp_gyro_y.to_le_bytes());
        packet[40..44].copy_from_slice(&self.uncomp_gyro_z.to_le_bytes());

        // Temp & Pressure [44-51]
        packet[44..48].copy_from_slice(&self.temp.to_le_bytes());
        packet[48..52].copy_from_slice(&self.press.to_le_bytes());

        // Mag [52-63]
        packet[52..56].copy_from_slice(&self.mag_x.to_le_bytes());
        packet[56..60].copy_from_slice(&self.mag_y.to_le_bytes());
        packet[60..64].copy_from_slice(&self.mag_z.to_le_bytes());

        // Accel [64-75]
        packet[64..68].copy_from_slice(&self.accel_x.to_le_bytes());
        packet[68..72].copy_from_slice(&self.accel_y.to_le_bytes());
        packet[72..76].copy_from_slice(&self.accel_z.to_le_bytes());

        // Gyro [76-87]
        packet[76..80].copy_from_slice(&self.gyro_x.to_le_bytes());
        packet[80..84].copy_from_slice(&self.gyro_y.to_le_bytes());
        packet[84..88].copy_from_slice(&self.gyro_z.to_le_bytes());

        // Quat [88-103]
        packet[88..92].copy_from_slice(&self.quat_x.to_le_bytes());
        packet[92..96].copy_from_slice(&self.quat_y.to_le_bytes());
        packet[96..100].copy_from_slice(&self.quat_z.to_le_bytes());
        packet[100..104].copy_from_slice(&self.quat_w.to_le_bytes());
        // Lin Accel [104-115]
        packet[104..108].copy_from_slice(&self.lin_accel_x.to_le_bytes());
        packet[108..112].copy_from_slice(&self.lin_accel_y.to_le_bytes());
        packet[112..116].copy_from_slice(&self.lin_accel_z.to_le_bytes());

        // Accel NED [116-127]
        packet[116..120].copy_from_slice(&self.accel_n.to_le_bytes());
        packet[120..124].copy_from_slice(&self.accel_e.to_le_bytes());
        packet[124..128].copy_from_slice(&self.accel_d.to_le_bytes());

         //crc [128-129];
        //let crc = calculate_crc(&p);
      // packet[128..130].copy_from_slice(&crc.to_le_bytes());

        packet[128] = 0xCC; // Dummy CRC
        packet[129] = 0xAA;
        packet
    }
}