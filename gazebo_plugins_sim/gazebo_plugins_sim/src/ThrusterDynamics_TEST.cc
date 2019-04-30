/***************************************************
 * Title: UUV Simulator
 * Author: The UUV Simulator Authors
 * Date: 2016
 * Availability: https://uuvsimulator.github.io/
***************************************************/

#include <gtest/gtest.h>

#include <gazebo_plugins_sim/Dynamics.hh>

std::shared_ptr<gazebo::Dynamics> DynamicsFromString(
    const std::string& description)
{
  std::stringstream stream;
  stream << "<sdf version='" << SDF_VERSION << "'>" << std::endl
         << "<model name='test_model'>" << std::endl
         << "<plugin name='test_plugin' filename='test_file.so'>" << std::endl
         << description
         << "</plugin>" << std::endl
         << "</model>" << std::endl
         << "</sdf>" << std::endl;

  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr dynSdf = sdfParsed.Root()->GetElement("model")
      ->GetElement("plugin")->GetElement("dynamics");

  std::shared_ptr<gazebo::Dynamics> dyn;
  dyn.reset(gazebo::DynamicsFactory::GetInstance().
             CreateDynamics(dynSdf));

  return dyn;
}

TEST(ThrusterDynamics, ZeroOrder)
{
  std::string description =
        "<dynamics> \n"
        "  <type>ZeroOrder</type> \n"
        "</dynamics>";

  std::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "ZeroOrder");

  EXPECT_EQ(10.0, dyn->update(10.0, 0.0));
  EXPECT_EQ(20.0, dyn->update(20.0, 0.2));
}

TEST(ThrusterDynamics, FirstOrder)
{
  std::string description =
        "<dynamics>\n"
        "  <type>FirstOrder</type>\n"
        "  <timeConstant>0.5</timeConstant>\n"
        "</dynamics>";

  std::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "FirstOrder");
  EXPECT_EQ(0.0, dyn->update(0.0, 0));
  EXPECT_NEAR(1-0.36787944, dyn->update(1.0, 0.5), 1e-5);
}

TEST(ThrusterDynamics, Yoerger)
{
  std::string description =
        "<dynamics> \n"
        "  <type>Yoerger</type>\n"
        "  <alpha>0.5</alpha>\n"
        "  <beta>0.5</beta>\n"
        "</dynamics>";

  std::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "Yoerger");
  EXPECT_EQ(0.0, dyn->update(0.0, 0));
  // TODO: Actually test dynamic behavior
}

TEST(ThrusterDynamics, Bessa)
{
  std::string description =
        "<dynamics> \n"
        "  <type>Bessa</type>\n"
        "  <Jmsp>0.5</Jmsp>\n"
        "  <Kv1>0.5</Kv1>\n"
        "  <Kv2>0.5</Kv2>\n"
        "  <Kt>0.5</Kt>\n"
        "  <Rm>0.5</Rm>\n"
        "</dynamics>";

  std::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "Bessa");
  EXPECT_EQ(0.0, dyn->update(0.0, 0));
  // TODO: Actually test dynamic behavior
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
