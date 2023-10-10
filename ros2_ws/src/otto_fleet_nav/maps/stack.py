import numpy as np
from PIL import Image

# Load your images into NumPy arrays
image1 = np.array(Image.open('map_demolevel1.pgm'))
image2 = np.array(Image.open('map_demolevel1.pgm'))
image3 = np.array(Image.open('map_demolevel1.pgm'))

# Check the dimensions of the images
height, width = image1.shape

# Stack the images vertically
stacked_image = np.vstack((image1, image2, image3))

# Save the resulting stacked image
stacked_image_pil = Image.fromarray(stacked_image)
stacked_image_pil.save('map_demolevel2.pgm')

# Display the stacked image
stacked_image_pil.show()

