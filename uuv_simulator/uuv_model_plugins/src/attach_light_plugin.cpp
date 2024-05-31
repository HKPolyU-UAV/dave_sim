#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
  class attach_light_plugin : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Create a new light
      sdf::SDF lightSDF;
      lightSDF.SetFromString(
        "<sdf version ='1.5'>\
          <light name='light_source1' type='spot'>\
            <pose>0.25 0 0.25 0 -0.7854 0</pose>\
            <attenuation>\
              <range>30</range>\
              <linear>0.05</linear>\
            </attenuation>\
            <diffuse>1 0 0 1</diffuse>\
            <specular>1 0 0 1</specular>\
            <spot>\
              <inner_angle>0.3</inner_angle>\
              <outer_angle>0.35</outer_angle>\
              <falloff>1</falloff>\
            </spot>\
            <direction>0 0 -1</direction>\
          </light>\
        </sdf>");
      // Get a list of all lights in the world
      auto lights = _model->GetWorld()->Lights();

      // Find the light you're interested in
      physics::LightPtr light;
      for (auto current_light : lights)
      {
        if (current_light->GetName() == "light_source1")
        {
          light = current_light;
          break;
        }
      }
      // Parse the light SDF
      sdf::ElementPtr lightElement = lightSDF.Root()->GetElement("light");
      // Attach the light to the BlueROV2
      light->SetParent(_model->GetLink("${namespace}/base_link"));
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(attach_light_plugin)
}