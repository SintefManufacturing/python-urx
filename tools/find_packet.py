from urx import ursecmon

if __name__ == "__main__":
    f = open("packets.bin", "rb")
    s = open("packet.bin", "wb")
    data = f.read(99999)
    parser = ursecmon.ParserUtils()
    p, rest = parser.find_first_packet(data)
    print(len(p))
    p, rest = parser.find_first_packet(rest)
    print(len(p))
    s.write(p)
    p, rest = parser.find_first_packet(rest)
    print(len(p))
