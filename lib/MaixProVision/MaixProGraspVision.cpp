#include "MaixProGraspVision.h"

MaixProGraspVision::MaixProGraspVision(
    maixcam::MaixVisionService &vision)
    : _vision(vision)
{
}

void MaixProGraspVision::begin()
{
    stop();
}

bool MaixProGraspVision::startTracking(uint8_t color)
{
    if (color < 1 || color > 4)
        return false;

    _targetColor = color;
    _observationReady = false;
    _token = _vision.request(maixcam::MODE_GRAB, color);
    return true;
}

void MaixProGraspVision::update()
{
    maixcam::MaixVisionService::Observation frame;
    if (!_vision.readNew(_token, _cursor, frame))
        return;

    // 目标丢失帧的targetId为0，与原逻辑一致只保留本颜色的命中帧。
    if (frame.detection.targetId != _targetColor)
        return;

    _observation.found = frame.detection.found;
    _observation.dx = frame.detection.dx;
    _observation.dy = frame.detection.dy;
    _observation.quality = frame.detection.quality;
    _observation.receivedMs = frame.receivedMs;
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
    _vision.release(_token);
    _token = 0;
    _observationReady = false;
}

bool MaixProGraspVision::faulted() const
{
    if (_token == 0)
        return false;

    const maixcam::MaixVisionService::Status status =
        _vision.status(_token);
    return status == maixcam::MaixVisionService::Status::Rejected ||
           status == maixcam::MaixVisionService::Status::Failed ||
           status == maixcam::MaixVisionService::Status::Released;
}
