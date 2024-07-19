width = 600
height = 600
border_thickness = 10  # Adjust this value to change the thickness of the border

with open("simple_map.pgm", "w") as f:
    f.write("P2\n")  # PGM format
    f.write(f"{width} {height}\n")
    f.write("255\n")  # Max grayscale value

    for y in range(height):
        for x in range(width):
            # Check if the current pixel is within the border area
            if (x < border_thickness or x >= width - border_thickness or
                y < border_thickness or y >= height - border_thickness):
                f.write("0 ")  # Black border
            elif 200 < x < 400 and 200 < y < 400:
                f.write("0 ")  # Black obstacle
            else:
                f.write("255 ")  # White free space
        f.write("\n")

