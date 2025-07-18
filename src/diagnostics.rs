use bitfield_struct::bitfield;

/// Struct for DM1 Active Diagnostic Messages Packet
#[bitfield(u64)]
pub struct DM1Packet {
    #[bits(2)]
    pub protect_lamp: Lamp,
    #[bits(2)]
    pub amber_warning: Lamp,
    #[bits(2)]
    pub red_stop: Lamp,
    #[bits(2)]
    pub malfunction: Lamp,
    #[bits(2, default = FlashLamp::DoNotFlash)]
    pub protect_lamp_flash: FlashLamp,
    #[bits(2, default = FlashLamp::DoNotFlash)]
    pub amber_warning_flash: FlashLamp,
    #[bits(2, default = FlashLamp::DoNotFlash)]
    pub red_stop_flash: FlashLamp,
    #[bits(2, default = FlashLamp::DoNotFlash)]
    pub malfunction_flash: FlashLamp,
    /// J1939-73 requires of there is no malfunction these should be zeroed
    #[bits(19, default = 0x000000)]
    pub spn: u32,
    /// J1939-73 requires of there is no malfunction these should be zeroed
    /// It is unfortunate here cause it is addressed to a fault
    #[bits(5, default = FMI::DataValidAboveNormalOpRange)]
    pub fmi: FMI,
    /// J1939-73 requires of there is no malfunction these should be zeroed
    #[bits(7, default = 0x00)]
    pub occurence_count: u8,
    /// J1939-73 requires of there is no malfunction these should be zeroed
    #[bits(1, default = false)]
    pub spn_conversion_method_is_old: bool,
    /// J1939-73 requires of there is no malfunction these should be oned.
    #[bits(16, default = 0xffff)]
    pub reserved: u16,
}

#[repr(u8)]
#[derive(Default, Debug, Clone, Copy)]
pub enum Lamp {
    #[default]
    Off = 0,
    On = 1,
    NotAvailable0 = 2,
    NotAvailable1 = 3,
    Error = 4,
}

#[repr(u8)]
#[derive(Default, Debug, Clone, Copy)]
pub enum FlashLamp {
    SlowFlash = 0,
    FastFlash = 1,
    Reserved = 2,
    #[default]
    DoNotFlash = 3,
    Error = 4,
}

impl Lamp {
    // This has to be a const fn
    pub const fn into_bits(self) -> u8 {
        self as u8
    }

    pub const fn from_bits(value: u8) -> Self {
        match value {
            0 => Lamp::Off,
            1 => Lamp::On,
            2 => Lamp::NotAvailable0,
            3 => Lamp::NotAvailable1,
            _ => Lamp::Error,
        }
    }
}

impl FlashLamp {
    // This has to be a const fn
    pub const fn into_bits(self) -> u8 {
        self as u8
    }

    pub const fn from_bits(value: u8) -> Self {
        match value {
            0 => FlashLamp::SlowFlash,
            1 => FlashLamp::FastFlash,
            2 => FlashLamp::Reserved,
            3 => FlashLamp::DoNotFlash,
            _ => FlashLamp::Error,
        }
    }
}

/// Failure Mode Indicator
#[repr(u8)]
#[derive(Default, Debug, Clone, Copy)]
pub enum FMI {
    DataValidAboveNormalOpRange,
    DataValidButBelowNormalOpRange,
    DataErraticIntermittentOrIncorrect,
    VoltageAboveNormalOrShortedToHighSource,
    VoltageBelowNormalOrShortedToLowSource,
    CurrentBelowNormalOrOpenCircuit,
    CurrentAboveNormalOrGroundedCircuit,
    MechanicalSystemNotRespondingOrOutOfAdjustment,
    AbnormalFrequencyOrPulseWidthOrPeriod,
    AbnormalUpdateRate,
    AbnormalRateofChange,
    RootCauseNotKnown,
    BadIntelligentDeviceOrComponent,
    OutOfCalibration,
    SpecialInstructions1,
    DataValidButAboveNormalOpRangeLeastSevere,
    DataValidButAboveNormalOpRangeModeratelySevere,
    DataValidButBelowNormalOpRangeLeastSevere,
    DataValidButBelowNormalOpRangeModeratelySevere,
    ReceivedNetworkDataInError,
    DataDriftedHigh,
    DataDriftedLow,
    SpecialInstructions2,
    RequestDM60ForInfo,
    Reserved1,
    Reserved2,
    Reserved3,
    Reserved4,
    Reserved5,
    Reserved6,
    Reserved7,
    #[default]
    ConditionExists,
    Error,
}

/// Ser/de implementation of Override Mode enum
impl FMI {
    // This has to be a const fn
    pub const fn into_bits(self) -> u8 {
        self as u8
    }

    pub const fn from_bits(value: u8) -> Self {
        match value {
            0 => FMI::DataValidAboveNormalOpRange,
            1 => FMI::DataValidButBelowNormalOpRange,
            2 => FMI::DataErraticIntermittentOrIncorrect,
            3 => FMI::VoltageAboveNormalOrShortedToHighSource,
            4 => FMI::VoltageBelowNormalOrShortedToLowSource,
            5 => FMI::CurrentBelowNormalOrOpenCircuit,
            6 => FMI::CurrentAboveNormalOrGroundedCircuit,
            7 => FMI::MechanicalSystemNotRespondingOrOutOfAdjustment,
            8 => FMI::AbnormalFrequencyOrPulseWidthOrPeriod,
            9 => FMI::AbnormalUpdateRate,
            10 => FMI::AbnormalRateofChange,
            11 => FMI::RootCauseNotKnown,
            12 => FMI::BadIntelligentDeviceOrComponent,
            13 => FMI::OutOfCalibration,
            14 => FMI::SpecialInstructions1,
            15 => FMI::DataValidButAboveNormalOpRangeLeastSevere,
            16 => FMI::DataValidButAboveNormalOpRangeModeratelySevere,
            17 => FMI::DataValidButBelowNormalOpRangeLeastSevere,
            18 => FMI::DataValidButBelowNormalOpRangeModeratelySevere,
            19 => FMI::ReceivedNetworkDataInError,
            20 => FMI::DataDriftedHigh,
            21 => FMI::DataDriftedLow,
            22 => FMI::SpecialInstructions2,
            23 => FMI::RequestDM60ForInfo,
            24 => FMI::Reserved1,
            25 => FMI::Reserved2,
            26 => FMI::Reserved3,
            27 => FMI::Reserved4,
            28 => FMI::Reserved5,
            29 => FMI::Reserved6,
            30 => FMI::Reserved7,
            31 => FMI::ConditionExists,
            _ => FMI::Error,
        }
    }
}

/// Connection Management Packet (used for Multipacket Transmission)
/// Only to
#[bitfield(u64)]
struct TPCMPacket {
    #[bits(8)]
    control_byte: TPCMPacketType,
    #[bits(16)]
    message_size: u16,
    #[bits(8)]
    packet_count: u8,
    #[bits(8, default = 0xff)]
    reserved: u8,
    #[bits(24)]
    pgn: u32,
}

/// Packet Types for the TP.CM packets.
#[derive(Debug)]
pub enum TPCMPacketType {
    BAM = 32,
    RTS = 16,
    CTS = 17,
    ACK = 19,
    Error = 0xFF,
}

impl TPCMPacketType {
    const fn into_bits(self) -> u8 {
        self as u8
    }
    const fn from_bits(value: u64) -> Self {
        match value {
            32 => TPCMPacketType::BAM,
            36 => TPCMPacketType::RTS,
            17 => TPCMPacketType::CTS,
            19 => TPCMPacketType::ACK,
            _ => TPCMPacketType::Error,
        }
    }
}
