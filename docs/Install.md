# Install

[README](../README.md) · [Sample OpMode](SampleOpMode.md) · [Troubleshooting](Troubleshooting.md)

Current JitPack version: **1.0.0** (must match a [git tag](https://github.com/IamAki123/EasyOBJD/releases)).

## What you need

- An FTC SDK **10.x or 11.x** Android Studio project
- EasyOpenCV **1.7.x** (`implementation 'org.openftc:easyopencv:1.7.3'`)
- Android Studio’s Embedded JDK for the Gradle JVM (11+). EasyOBJD’s bytecode is Java 8

Copy-in samples under [`samples/`](../samples/) are **not** in the JitPack AAR.

Building *this* GitHub repo from the command line is different (JDK 17+, Android SDK): [Contributing](../CONTRIBUTING.md).

## Typical path: JitPack

Gradle will not resolve `com.github.IamAki123:EasyOBJD` until JitPack is a repository **and** the `implementation` line is in a `dependencies` block that TeamCode applies.

### 1. Repository

In the FTC project **root** `build.dependencies.gradle`, add JitPack to the existing `repositories` block:

```gradle
repositories {
    mavenCentral()
    google()
    maven { url = 'https://jitpack.io' }
}
```

If sync fails because `RepositoriesMode.FAIL_ON_PROJECT_REPOS` is set, add JitPack in the **root** `settings.gradle` instead.

Stock FTC SDK 11.1 `settings.gradle` usually does **not** set `FAIL_ON_PROJECT_REPOS`; `build.dependencies.gradle` is the usual place.

### 2. Dependency

In `TeamCode/build.gradle`:

```gradle
dependencies {
    implementation project(':FtcRobotController')
    implementation 'org.openftc:easyopencv:1.7.3'
    implementation 'com.github.IamAki123:EasyOBJD:1.0.0'
}
```

If `build.dependencies.gradle` already holds Pedro or other libraries, you can put the EasyOBJD `implementation` line in that file’s `dependencies` block instead.

### 3. Sync

File → Sync Project with Gradle Files.

**You are done when** the project syncs and Android Studio can autocomplete `org.firstinspires.ftc.easyobd.EasyOBJD`.

Next: copy [`EasyOBJDSample`](../samples/EasyOBJDSample.java) and continue in the [README](../README.md) (**2. Copy the sample**).

## Optional: local module

Use this if you want to edit EasyOBJD source next to TeamCode instead of JitPack.

1. Clone or copy this repository so it sits next to `TeamCode` (for example `YourFtcProject/EasyOBJD/`).
2. `include ':EasyOBJD'` in the FTC project’s root `settings.gradle`.
3. `implementation project(':EasyOBJD')` in `TeamCode/build.gradle`.
4. Sync with Android Studio’s Embedded JDK. The module name must match `include`.

---

[README](../README.md) · [Sample OpMode](SampleOpMode.md) · [API](API.md) · [Troubleshooting](Troubleshooting.md)
