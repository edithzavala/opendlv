/**
 * Copyright (C) 2017 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef LOGIC_KNOWLEDGE_V2VCAM_BUFFER_HPP
#define LOGIC_KNOWLEDGE_V2VCAM_BUFFER_HPP

#include <memory>
#include <string>
#include <vector>

namespace opendlv {
namespace logic {
namespace adaptation {

/**
 * @brief For storing bytes in a buffer.
 */
class Buffer {
  public:
    /**
     * @brief For reading a buffer.
     */
    class Iterator {
      public:
        explicit Iterator(const Buffer *);
        Iterator(const Iterator &) = delete;
        Iterator &operator=(const Iterator &) = delete;
        virtual ~Iterator();
        bool ReadBoolean();
        unsigned char ReadByte();
        std::shared_ptr<std::vector<unsigned char>> ReadBytes();
        std::vector<unsigned char> ReadBytesReversed(int );
        double ReadDouble();
        float ReadFloat();
        int ReadInteger();
        int ReadInteger24();
        long ReadLong();
        short ReadShort();
        std::string ReadString();
        void Reset();
        void ItReversed();


      private:
        void CheckOverflow(int);
        bool HasBytesLeft(int);

        const Buffer *m_outer_buffer;
        int m_read_pos;


        bool m_itreversed = false;
    };



    Buffer();
    explicit Buffer(const std::vector<unsigned char> &);
    Buffer(const Buffer &) = delete;
    Buffer &operator=(const Buffer &) = delete;
    virtual ~Buffer();
    void AppendBoolean(bool);
    void AppendByte(unsigned char);
    void AppendBytes(const std::vector<unsigned char> &);
    void AppendBytesRaw(const std::vector<unsigned char> &);
    void AppendDouble(double);
    void AppendFloat(float);
    void AppendInteger(int);
    void AppendLong(long);
    void AppendShort(short);
    void AppendString(const std::string &);
    void AppendStringRaw(const std::string &);
    const std::vector<unsigned char> &GetBytes() const;
    std::shared_ptr<Buffer::Iterator> GetIterator() const;
    int GetSize() const;
    unsigned char ReadByte(int) const;
    void Reversed();


  private:
    void Append(const std::vector<unsigned char> &);

    std::vector<unsigned char> m_bytes;


    bool m_reversed = false;

};

}
}
}

#endif
