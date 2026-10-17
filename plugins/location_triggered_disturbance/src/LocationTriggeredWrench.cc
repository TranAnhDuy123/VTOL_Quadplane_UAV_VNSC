#include <chrono>
#include <memory>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <sdf/Element.hh>

namespace uav
{
class LocationTriggeredWrench final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager & /*_eventMgr*/) override
  {
    gz::sim::Model model(_entity);
    if (!model.Valid(_ecm))
    {
      gzerr << "[LocationTriggeredWrench] Plugin must be attached to a model.\n";
      return;
    }

    const sdf::ElementPtr sdfConfig = _sdf ? _sdf->Clone() : nullptr;

    this->linkName = this->Read<std::string>(
        sdfConfig, "link_name", "base_link");
    this->linkEntity = model.LinkByName(_ecm, this->linkName);
    if (this->linkEntity == gz::sim::kNullEntity)
    {
      gzerr << "[LocationTriggeredWrench] Link [" << this->linkName
            << "] was not found.\n";
      return;
    }

    this->xMin = this->Read<double>(sdfConfig, "x_min", 371.5);
    this->xMax = this->Read<double>(sdfConfig, "x_max", 416.5);
    this->yMin = this->Read<double>(sdfConfig, "y_min", 925.0);
    this->yMax = this->Read<double>(sdfConfig, "y_max", 965.0);
    this->zMin = this->Read<double>(sdfConfig, "z_min", 35.0);
    this->zMax = this->Read<double>(sdfConfig, "z_max", 70.0);
    this->rollTorqueNm = this->Read<double>(
        sdfConfig, "roll_torque_nm", 0.60);

    if (!(this->xMin < this->xMax && this->yMin < this->yMax &&
          this->zMin < this->zMax))
    {
      gzerr << "[LocationTriggeredWrench] Invalid zone or duration.\n";
      this->linkEntity = gz::sim::kNullEntity;
      return;
    }

    gzmsg << "[LocationTriggeredWrench] Armed for link [" << this->linkName
          << "]; zone x=[" << this->xMin << ", " << this->xMax
          << "], y=[" << this->yMin << ", " << this->yMax
          << "], z=[" << this->zMin << ", " << this->zMax
          << "]; body-roll torque=" << this->rollTorqueNm
          << " N.m while the aircraft remains inside the zone.\n";
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || this->linkEntity == gz::sim::kNullEntity || this->done)
      return;

    const auto pose = gz::sim::worldPose(this->linkEntity, _ecm);
    const auto &p = pose.Pos();
    const double simTimeS =
        std::chrono::duration<double>(_info.simTime).count();

    const bool inside =
        p.X() >= this->xMin && p.X() <= this->xMax &&
        p.Y() >= this->yMin && p.Y() <= this->yMax &&
        p.Z() >= this->zMin && p.Z() <= this->zMax;

    if (inside && !this->active)
    {
      this->active = true;
      this->startTimeS = simTimeS;
      gzmsg << "[LocationTriggeredWrench] ON  t=" << simTimeS
            << " s, position=(" << p.X() << ", " << p.Y()
            << ", " << p.Z() << ").\n";
    }

    if (!inside)
    {
      if (this->active)
      {
        this->active = false;
        this->done = true;
        gzmsg << "[LocationTriggeredWrench] OFF t=" << simTimeS
              << " s, position=(" << p.X() << ", " << p.Y()
              << ", " << p.Z() << "), exposure="
              << simTimeS - this->startTimeS
              << " s; trigger locked until simulation restart.\n";
      }
      return;
    }

    // Model +X is the aircraft body-roll axis. Rotate it into world
    // coordinates so the applied moment remains a pure body-roll disturbance
    // even if the aircraft heading is not perfectly aligned with the route.
    const gz::math::Vector3d bodyRollAxisWorld =
        pose.Rot().RotateVector(gz::math::Vector3d::UnitX);
    const gz::math::Vector3d torqueWorld =
        bodyRollAxisWorld * this->rollTorqueNm;

    gz::sim::Link link(this->linkEntity);
    link.AddWorldWrench(
        _ecm, gz::math::Vector3d::Zero, torqueWorld);
  }

  private: template<typename T>
  T Read(const sdf::ElementPtr &_sdf,
         const std::string &_name, const T &_defaultValue)
  {
    if (_sdf && _sdf->HasElement(_name))
      return _sdf->Get<T>(_name);
    return _defaultValue;
  }

  private: std::string linkName{"base_link"};
  private: gz::sim::Entity linkEntity{gz::sim::kNullEntity};
  private: double xMin{371.5};
  private: double xMax{416.5};
  private: double yMin{925.0};
  private: double yMax{965.0};
  private: double zMin{35.0};
  private: double zMax{70.0};
  private: double rollTorqueNm{0.60};
  private: double startTimeS{0.0};
  private: bool active{false};
  private: bool done{false};
};
}

GZ_ADD_PLUGIN(
    uav::LocationTriggeredWrench,
    gz::sim::System,
    uav::LocationTriggeredWrench::ISystemConfigure,
    uav::LocationTriggeredWrench::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    uav::LocationTriggeredWrench,
    "uav::LocationTriggeredWrench")
