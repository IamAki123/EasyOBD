# EasyOBD

FTC EasyOpenCV pipeline that finds yellow game-piece **clusters** and reports each cluster’s camera-relative **X / Y** in inches.

JitPack: `com.github.IamAki123:EasyOBD:0.1`

## Install

In `build.dependencies.gradle` (or your root repositories):

```gradle
maven { url = 'https://jitpack.io' }
```

In `TeamCode/build.gradle`:

```gradle
implementation 'com.github.IamAki123:EasyOBD:0.1'
implementation 'org.openftc:easyopencv:1.7.3'
```

Or, while developing next to this repo:

```gradle
implementation project(':EasyOBD')
```

## Use

Copy [`samples/EasyOBDSample.java`](samples/EasyOBDSample.java) into TeamCode, or:

```java
EasyOBDCreation pipeline = EasyOBD.createPipeline();
webcam.setPipeline(pipeline);

// After streaming:
int clusters = pipeline.getClusterCount();
int balls = pipeline.getBallCount();
for (ClusterInfo cluster : pipeline.getClusters()) {
    // cluster.id is 1..n, left to right
    // cluster.x = inches right of the lens
    // cluster.y = inches forward of the lens
}
```

Live HSV tune on gamepad1: **D-pad up** widens yellow, **D-pad down** tightens. Tuned starting range:

- Lower: `21, 95, 85`
- Upper: `35, 255, 255`

## What it does

1. HSV yellow mask (everything else black)
2. 12×12 grid, touching white cells become a cluster
3. Circular / arc ball detection (rejects the empty valley between two balls)
4. Floor-plane X/Y from camera height + pitch

Requires a webcam named `Webcam 1` unless you change the sample.
