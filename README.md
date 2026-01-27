# edge-tts-rbnx

Edge TTS package for Robonix text-to-speech primitive capability.

## Description

This package provides a ROS2 primitive for text-to-speech synthesis using Microsoft Edge TTS. It receives text messages and synthesizes them into speech audio, which is then played through the system audio output.

## Primitive

- **Name**: `prm::speech.tts`
- **Input**: `text` (std_msgs/String) - Text to be synthesized
- **Output**: `status` (std_msgs/Bool) - Success status of the operation

## Topics

- **Input Topic**: `speech/tts/text` (std_msgs/String)
- **Output Topic**: `speech/tts/status` (std_msgs/Bool)

## Requirements

- Python 3.10+
- ROS2 Humble
- `edge-tts` Python package
- `pydub` Python package
- System audio support (PulseAudio or ALSA)

## Installation

The package is built and managed by the Robonix package system:

```bash
rbnx package build edge-tts-rbnx
```

## Usage

The primitive can be used in Robonix recipes by including it in the package list:

```yaml
packages:
  - name: edge-tts-rbnx
    primitives:
      - prm::speech.tts
```

### Publishing Text to Synthesize

```bash
ros2 topic pub /speech/tts/text std_msgs/String "data: 'Hello, this is a test'"
```

### Checking Status

```bash
ros2 topic echo /speech/tts/status
```

## Configuration

The node supports the following ROS2 parameters:

- `voice` (default: `zh-CN-XiaoyiNeural`) - Voice to use for synthesis
- `volume` (default: `30%`) - Audio output volume
- `device` (default: `""`) - Audio device to use (empty for auto-detection)

## License

Apache-2.0

## Maintainer

wheatfox <wheatfox17@icloud.com>
