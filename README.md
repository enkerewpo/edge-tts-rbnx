# edge-tts-rbnx

Edge TTS package for Robonix text-to-speech primitive and **speak skill** (for RTDL skill-only invocation).

## Description

This package provides:
1. A ROS2 **primitive** `prm::speech.tts` for TTS synthesis.
2. A **skill** `skl::speak` that wraps the primitive so RTDL (and other skill-only callers) can invoke TTS without calling primitives directly.

## Primitive

- **Name**: `prm::speech.tts`
- **Input**: `text` (std_msgs/String) - Text to be synthesized
- **Output**: `status` (std_msgs/Bool) - Success status of the operation

## Skill (RTDL / skill-only)

- **Name**: `skl::speak`
- **Start topic**: `/robot1/skill/speak/start` (std_msgs/String, JSON)
- **Status topic**: `/robot1/skill/speak/status` (std_msgs/String, JSON)
- **Start args**: `{"text": "string (required)", "voice": "string (optional)"}`  
  The skill resolves `prm::speech.tts` via QueryPrimitive, publishes `text` to the primitive, and reports skill status when TTS completes.

## Topics

- **Primitive input**: `speech/tts/text` (std_msgs/String)
- **Primitive output**: `speech/tts/status` (std_msgs/Bool)

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
