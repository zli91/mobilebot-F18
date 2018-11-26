"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class message_recieved_t(object):
    __slots__ = ["utime", "creation_time", "channel"]

    def __init__(self):
        self.utime = 0
        self.creation_time = 0
        self.channel = ""

    def encode(self):
        buf = BytesIO()
        buf.write(message_recieved_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qq", self.utime, self.creation_time))
        __channel_encoded = self.channel.encode('utf-8')
        buf.write(struct.pack('>I', len(__channel_encoded)+1))
        buf.write(__channel_encoded)
        buf.write(b"\0")

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != message_recieved_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return message_recieved_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = message_recieved_t()
        self.utime, self.creation_time = struct.unpack(">qq", buf.read(16))
        __channel_len = struct.unpack('>I', buf.read(4))[0]
        self.channel = buf.read(__channel_len)[:-1].decode('utf-8', 'replace')
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if message_recieved_t in parents: return 0
        tmphash = (0x8f572e30405376e3) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if message_recieved_t._packed_fingerprint is None:
            message_recieved_t._packed_fingerprint = struct.pack(">Q", message_recieved_t._get_hash_recursive([]))
        return message_recieved_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

