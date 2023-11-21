//! TT21100 Multi-Touch Touchscreen Controller

#![no_std]

use core::{array::TryFromSliceError, fmt::Debug};

use bondrewd::Bitfields;
use embedded_hal_async::{digital::Wait, i2c::I2c};

// Default I²C address for the TT21100
const I2C_ADDR: u8 = 0x24;

/// Any type of error which may occur while interacting with the device
#[derive(Debug)]
pub enum Error<E> {
    /// Some error originating from the communication bus
    BusError(E),
    /// The message length did not match the expected value
    InvalidMessageLen(usize),
    /// Reading a GPIO pin resulted in an error
    IOError,
    /// Tried to read a touch point, but no data was available
    NoDataAvailable,
    /// Error converting a slice to an array
    TryFromSliceError,
}

impl<E> From<TryFromSliceError> for Error<E> {
    fn from(_: TryFromSliceError) -> Self {
        Self::TryFromSliceError
    }
}

/// An event emitted by the device
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// A touch event
    Touch {
        report: TouchReport,
        touches: (Option<TouchRecord>, Option<TouchRecord>),
    },
    /// A button press event
    Button(ButtonRecord),
}

/// Prelude data for one or more touch events
#[derive(Debug, Clone, Copy, PartialEq, Eq, Bitfields)]
#[bondrewd(default_endianness = "le")]
pub struct TouchReport {
    /// Total length of the data; should be 7, 17, or 27
    pub data_len: u16,
    /// ID of the report
    pub report_id: u8,
    /// Timestamp
    pub time_stamp: u16,
    #[bondrewd(bit_length = 2)]
    padding0: u8,
    #[bondrewd(bit_length = 1)]
    pub large_object: u8,
    #[bondrewd(bit_length = 5)]
    pub record_num: u8,
    #[bondrewd(bit_length = 2)]
    pub report_counter: u8,
    #[bondrewd(bit_length = 3)]
    padding1: u8,
    #[bondrewd(bit_length = 3)]
    pub noise_effect: u8,
}

/// Data for a touch event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Bitfields)]
#[bondrewd(default_endianness = "le")]
pub struct TouchRecord {
    #[bondrewd(bit_length = 5)]
    padding0: u8,
    #[bondrewd(bit_length = 3)]
    pub touch_type: u8,
    #[bondrewd(bit_length = 1)]
    pub tip: u8,
    #[bondrewd(bit_length = 2)]
    pub event_id: u8,
    #[bondrewd(bit_length = 5)]
    pub touch_id: u8,
    pub x: u16,
    pub y: u16,
    pub pressure: u8,
    pub major_axis_length: u16,
    pub orientation: u8,
}

/// Data for a button press event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Bitfields)]
#[bondrewd(default_endianness = "le")]
pub struct ButtonRecord {
    /// Length of the record; always `14`
    pub length: u16,
    /// ID of the report; always `3`
    pub report_id: u8,
    /// Timestamp in units of 100us
    pub time_stamp: u16,
    /// Button value; only use bits[3..0]
    pub btn_val: u8,
    /// Button signals
    pub btn_signal: [u16; 4],
}

/// TT21100 driver
pub struct TT21100<I2C, IRQ> {
    /// Underlying I²C peripheral
    i2c: I2C,
    /// Interrupt pin
    irq: IRQ,
}

impl<I2C, IRQ, E> TT21100<I2C, IRQ>
where
    I2C: I2c<Error = E>,
    IRQ: Wait,
    E: Debug,
{
    /// Create a new instance of the driver and initialize the device
    pub fn new(i2c: I2C, irq: IRQ) -> Self {
        Self { i2c, irq }
    }

    /// Is there data available to read from the device?
    pub async fn data_available(&mut self) -> Result<(), Error<E>> {
        self.irq.wait_for_low().await.map_err(|_| Error::IOError)
    }

    /// Read an event from the device
    ///
    /// There are two types of events, [Event::Touch] and [Event::Button].
    pub async fn event(&mut self) -> Result<Event, Error<E>> {
        let message_length = self.read_message_length().await?;

        let mut data = [0u8; 32];
        self.read_bytes(&mut data[0..][..message_length]).await?;

        match message_length {
            2 => Err(Error::NoDataAvailable),
            7 | 17 | 27 => touch_event(&data[0..][..message_length]),
            14 => button_event(&data[0..][..message_length]),
            n => Err(Error::InvalidMessageLen(n)),
        }
    }

    // -----------------------------------------------------------------------
    // PRIVATE

    async fn read_message_length(&mut self) -> Result<usize, Error<E>> {
        let mut buffer = [0u8; 2];
        self.read_bytes(&mut buffer).await?;

        let message_length = u16::from_le_bytes(buffer);

        Ok(message_length as usize)
    }

    async fn read_bytes(&mut self, buffer: &mut [u8]) -> Result<(), Error<E>> {
        self.i2c
            .write_read(I2C_ADDR, &[], buffer)
            .await
            .map_err(|e| Error::BusError(e))
    }
}

fn touch_event<E>(message: &[u8]) -> Result<Event, Error<E>>
where
    E: Debug,
{
    debug_assert!(message.len() == 7 || message.len() == 17 || message.len() == 27);

    let report = message[0..][..7].try_into()?;
    let report = TouchReport::from_bytes(report);

    let record0 = if message.len() >= 17 {
        let record = message[7..][..10].try_into()?;
        let record = TouchRecord::from_bytes(record);

        Some(record)
    } else {
        None
    };

    let record1 = if message.len() == 27 {
        let record = message[17..][..10].try_into()?;
        let record = TouchRecord::from_bytes(record);

        Some(record)
    } else {
        None
    };

    Ok(Event::Touch {
        report,
        touches: (record0, record1),
    })
}

fn button_event<E>(message: &[u8]) -> Result<Event, Error<E>>
where
    E: Debug,
{
    debug_assert_eq!(message.len(), 14);

    let message = message.try_into()?;
    let record = ButtonRecord::from_bytes(message);

    Ok(Event::Button(record))
}
