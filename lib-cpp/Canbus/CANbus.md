# Using the CANbus Library
## prerequisits:
Before you can use the library the following needs to be installed:
- Can4Linux
- Advantech CAN library `advcan`

## Required structures:
```cpp  
struct canmsg_t {
  int flags;                            //some flags stating errors etc
  int cob;                              // some more error handling from can4linux.h
  unsigned long id;                     //the id of the send message
  struct timeval timestamp;             //the timestamp of the received message
  short int length;                     // Length of the message should match CAN_MSG_LENGTH
  unsigned char data[CAN_MSG_LENGTH];   // The data in an array of Bytes(max 8)
}

struct CanStatus{
  // Baudrate of the CANbus in bits so 250kb/s is 250000
  unsigned int baudrate;    
  
  // Name of the device ex: "can0", "can1" 
  char device[20];
  
  // Amount of buffer left before max length of buffer is gone
  // TODO(Sander): Implement the use of this
  unsigned int buffer_left;
  
  // Amount the total buffer was initially set to
  // TODO(Sander): Implemenent the use of this
  unsigned int total_buffer;
  
  // Amount of times something was read two time without needing to.
  // TODO(Sander): Implement the use of this
  unsigned int double_reads;   
  
  // True if the CANbus is currently running, false otherwise
  bool status;
};
```
## Constructor:
```cpp
  /*
   * Initializes the canbus and starts it.
   * 
   * Arguments:
   *  char device_name[20]: ex. "can0", "can1"
   *  buffer_size: The max number of messages stored temporarily 
   * 
   * Example:
   *  CANbus canbus("can0", 100);
   *  
   *  canmsg_t receive_message;
   *  canbus.read(12458, receive_message);
   * 
   *  canmsg_t transmit_message;
   *  transmit_message.data[0] = 'a';
   *  transmit_message.data[1] = 'b'; \\etcetera
   *  transmit_message.id = 12548;
   *  transmit_message.length = 2;
   *  canbus.write(transmit_message);
   * 
   *  canbus.close(); 
   *  
   */
CANbus(char device_name[20], unsigned int buffer_size, unsigned int baudrate=250000);
```
## Methods:
```cpp

  /*
   * Initializes the canbus and starts it.
   * 
   * Arguments:
   *  char device_name[20]: ex. "can0", "can1"
   *  buffer_size: The max number of messages stored temporarily 
   * 
   * Example:
   *  CANbus canbus("can0", 100);
   *  
   *  canmsg_t read_message;
   *  canbus.read(12458, 
   */
  CANbus(char device_name[20], unsigned int buffer_size, unsigned int baudrate = 250000);


  /*
   * Looks in received messages if there is a message with that id.
   * If that it is the case writes that message to msg. 
   * 
   * Arguments:
   *  unsigned long msg_id: The message id you want to have the data from
   *  canmsg_t *buffer: A pointer to the buffer where the message will be written
   * 
   * Returns :
   *  -1: when failure (no msg id in list of msg_ids)
   *  0: when the message has been read before
   *  1: when read
   */
  int read(unsigned long msg_id, canmsg_t * const buffer);


  /*
   * Writes a can message to the canbus;
   * 
   * Arguments:
   *  cansmsg_t * const message: A can structure of the message you want to send
   *  force_send: boolean to set if a certain thing has to be send continiously
   * 
   * Returns:
   *  -1: Failure
   *  0: Nothing written
   *  1: Success
   */
  int write(canmsg_t * const message, bool force_send = false);


  /*
   * Starts the CANbus, it will open the device and stuff
   * 
   * Returns:
   *  -1: Start unsuccessful
   *  1: Success
   */
  int start();


  /*
   * Closes the CANbus,
   * 
   * Returns:
   *  -1: Close unsuccessful
   *  1: Success
   */
  int stop();


  /*
   * Gets the status of the Canbus
   */
  CanStatus status();

```
