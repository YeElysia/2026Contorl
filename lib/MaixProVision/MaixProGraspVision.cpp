#include "MaixProGraspVision.h"

#include "vision_config.h"

MaixProGraspVision::MaixProGraspVision(
    maixcam::MaixCamV2 &camera)
    : _camera(camera)
{
}

void MaixProGraspVision::begin()
{
    _begun = true;
    _camera.reset();
    _active = false;
    _faulted = false;
    _observationReady = false;
}

bool MaixProGraspVision::startTracking(uint8_t color)
{
    if (color < 1 || color > 4)
        return false;

    _targetColor = color;
    _observationReady = false;
    _faulted = false;
    _active = true;
    _camera.setTarget(maixcam::MODE_GRAB, color);
    return true;
}

void MaixProGraspVision::update()
{
    // 未执行抓取时不读取共享相机，避免消费圆环对准器的应答和图像。
    if (!_active)
        return;

    _camera.poll();

    uint8_t command = 0;
    uint8_t result = 0;
    uint8_t mode = 0;
    uint8_t selector = 0;
    if (_camera.takeAck(command, result, mode, selector) &&
        command == maixcam::CMD_SET_TARGET &&
        result != 0)
    {
        _faulted = true;
    }

    maixcam::Detection detection;
    if (!_camera.takeDetection(detection))
        return;

    if (detection.mode != maixcam::MODE_GRAB ||
        detection.targetId != _targetColor)
    {
        return;
    }

    _observation.found = detection.found;
    _observation.dx = detection.dx;
    _observation.dy = detection.dy;
    _observation.quality = detection.quality;
    _observation.receivedMs = millis();
    _observationReady = true;
}

bool MaixProGraspVision::takeObservation(
    GraspObservation &observation)
{
    if (!_observationReady)
        return false;

    observation = _observation;
    _observationReady = false;
    return true;
}

void MaixProGraspVision::stop()
{
    _active = false;
    _observationReady = false;
    if (_begun)
        _camera.reset();
}

bool MaixProGraspVision::faulted() const
{
    return _faulted;
}
