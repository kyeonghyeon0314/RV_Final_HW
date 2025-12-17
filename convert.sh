#!/bin/bash

# Check if the images directory exists, if not, create it
if [ ! -d "images" ]; then
  mkdir images
fi

# Convert video to frames (select 1 out of every 10 frames)
echo "Converting video/IMG_7694.MOV to frames (1/10th) in the 'images' directory..."
ffmpeg -i video/IMG_7694.MOV -vf "select=not(mod(n\,10))" -vsync vfr images/frame_%04d.jpg

echo "Conversion complete."