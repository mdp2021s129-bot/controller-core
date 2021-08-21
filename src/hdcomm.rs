/// Helpers for performing host-device communication using the hdcomm protocol.
use hdcomm_core::message::Message;
use heapless::Deque;

/// A `MessageQueue` helps to queue messages for transmission.
pub type MessageQueue<const N: usize> = Deque<Message, N>;
