from struct import unpack
from enum import Enum

from _ import SSL_WrapperPacket
from _ import Referee

# only types that logs contain
class SSLMessageType(Enum):
    MESSAGE_BLANK = 0
    MESSAGE_UNKNOWN = 1
    MESSAGE_SSL_VISION_2010 = 2  # SSL_WrapperPacket
    MESSAGE_SSL_REFBOX_2013 = 3  # SSL_Referee
    MESSAGE_SSL_VISION_2014 = 4
    MESSAGE_SSL_VISION_TRACKER_2020 = 5
    MESSAGE_SSL_INDEX_2021 = 6

class MessageType(Enum):
    VISION = 0
    REFEREE = 1

class SSLLogReader():
    FILE_HEADER_SIZE = 16
    DATA_HEADER_SIZE = 16

    def __init__(self, filepath):
        self.filepath = filepath
        # Maybe lazy read later but for now just brute force
        self.messages = []
        with open(filepath, 'rb') as file:
            self.name, self.version = unpack(
                ">12si", file.read(self.FILE_HEADER_SIZE))
            for message_header_buf in iter(lambda: file.read(self.DATA_HEADER_SIZE), ''):
                timestamp, message_type, message_size = unpack(
                    ">q2i", message_header_buf)
                payload_buf = file.read(message_size)

                vision_packet = SSL_WrapperPacket()
                referee_packet = Referee()

                if message_type == SSLMessageType.MESSAGE_BLANK:
                    pass
                    # ignore
                elif message_type == SSLMessageType.MESSAGE_UNKNOWN:
                    # these were the only ones the log player supported so really
                    # unsure what the other types are for so just took their logic directly in case they expand it ever
                    if vision_packet.ParseFromString(payload_buf):
                        self.messages.append((timestamp, MessageType.VISION, vision_packet))
                    elif referee_packet.ParseFromString(payload_buf):
                        self.messages.append((timestamp, MessageType.REFEREE, referee_packet))
                    else:
                        print(
                            "Error unsupported or corrupt packet found in log file!")
                elif message_type == SSLMessageType.MESSAGE_SSL_VISION_2010:
                    print("MESSAGE_SSL_VISION_2010")
                elif message_type == SSLMessageType.MESSAGE_SSL_REFBOX_2013:
                    print("MESSAGE_SSL_REFBOX_2013")
                elif message_type == SSLMessageType.MESSAGE_SSL_VISION_2014:
                    print("MESSAGE_SSL_VISION_2014")
                else:
                    print("Error unsupported or corrupt packet found in log file!")
