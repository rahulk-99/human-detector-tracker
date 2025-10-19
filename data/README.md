# Test Data Directory

This directory is for storing test images and videos.

## Phase 0 (Current)

Phase 0 uses synthetic/mock frames, so no actual test data is required.

## Phase 1+ (Future)

Add sample videos and images for testing:

### Recommended Test Datasets

1. **MOT Challenge Dataset** (Multiple Object Tracking)
   - https://motchallenge.net/
   - Contains annotated pedestrian tracking sequences
   - Good for benchmarking tracking performance

2. **COCO Dataset** (Common Objects in Context)
   - https://cocodataset.org/
   - Contains thousands of images with person annotations
   - Good for detection testing

3. **Your Own Videos**
   - Record videos from your robot's camera
   - Capture various lighting conditions
   - Test with different crowd densities

### File Organization

```
data/
├── images/
│   ├── test_image_01.jpg
│   ├── test_image_02.jpg
│   └── ...
├── videos/
│   ├── indoor_walking.mp4
│   ├── outdoor_crowd.mp4
│   └── ...
└── annotations/
    ├── test_image_01.json
    └── ...
```

## Data Files (gitignored)

Large data files (images, videos) are not committed to git.
Add `.jpg`, `.png`, `.mp4`, `.avi` files to this directory locally.

## Sample Commands

### Process a single image
```bash
./build/app/shell-app --image data/images/test_image_01.jpg
```

### Process a video
```bash
./build/app/shell-app --video data/videos/indoor_walking.mp4
```

### Process live camera
```bash
./build/app/shell-app --camera 0
```

(Note: These commands are placeholders for Phase 1+)

