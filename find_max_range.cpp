#include <iostream>
#include <vector>
#include <utils.h>
#include <cmdparser.h>
#include <algorithm>

#include <particle_filter.h>

using namespace std;

int main(int argc, char* argv[]) {

  CmdParser cmd(argc, argv);

  cmd.add("robot_data_log");
  
  cmd.addGroup("General options:")
    .add("--k-beam-per-scan", "number of beam per LASER scan", "180");

  cmd.addGroup("Example usage: ./particle_filter data/log/robotdata2.log");

  if (!cmd.isOptionLegal())
    cmd.showUsageAndExit();

  string robot_data = cmd[1];

  Laser::kBeamPerScan = cmd["--k-beam-per-scan"];

  // Load data log
  int max_range = 0;
  auto sensor_msgs = parse_robot_data(robot_data);
  for (auto& sensor_msg : sensor_msgs) {
    if (sensor_msg->type() != SensorMsg::Laser)
      continue;

    Laser* laser = dynamic_cast<Laser*>(sensor_msg);
    max_range = std::max(max_range, *std::max_element(
	laser->ranges.begin(), laser->ranges.end()
      )
    );
  }

  cout << max_range << endl;

  return 0;
}

