#ifndef bytes_h
#define bytes_h

#include <stdint.h>

extern uint16_t read16le(uint8_t*const data);
extern uint32_t read32le(uint8_t*const data);

extern uint16_t read16be(uint8_t*const data);
extern uint32_t read24be(uint8_t*const data);
extern uint32_t read32be(uint8_t*const data);

template<typename T, unsigned int S> class Buffer final {
public:
  T buffer[S];
  int readPos = 0;
  int writePos = 0;

  constexpr int capacity() {
    return S;
  }

  void add(T data) {
    if (remaining()) {
      buffer[writePos++] = data;
    }
  }

  void copyFrom(T* data, const int len) {
    if (len <= remaining()) {
      memcpy(newData(), data, len);
      writePos += len;
    }
  }

  T read() {
    return buffer[readPos++];
  }

  void consumed(int size) {
    readPos += size;
  }

  void supplied(int size) {
    writePos += size;
  }

  T* data() {
    return &(buffer[readPos]);
  }

  T* newData() {
    return &(buffer[writePos]);
  }

  bool isEmpty() const {
    return writePos <= readPos;
  }

  int size() const {
    return writePos - readPos;
  }

  int remaining() const {
    return capacity() - writePos;
  }

  bool fitsWithin(int availableCapacity) const {
    return size() <= availableCapacity;
  }

  void reset() {
    readPos = 0;
    writePos = 0;
  }
};

#endif
