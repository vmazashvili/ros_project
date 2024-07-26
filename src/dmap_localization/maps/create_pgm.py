# create_pgm.py
width = 600
height = 600

with open("simple_map.pgm", "w") as f:
    f.write("P2\n")  # PGM format
    f.write(f"{width} {height}\n")
    f.write("255\n")  # Max grayscale value

    for y in range(height):
        for x in range(width):
            if 200 < x < 400 and 200 < y < 400:
                f.write("0 ")  # Black obstacle
            else:
                f.write("255 ")  # White free space
        f.write("\n")

