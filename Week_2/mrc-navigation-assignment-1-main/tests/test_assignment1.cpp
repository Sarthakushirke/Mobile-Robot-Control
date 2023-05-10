#include <gtest/gtest.h>
#include <cmath>

#include "./Planner/planning.h"
#include "./tools.h"


int main_argc;
char** main_argv;

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  main_argc = argc;
  main_argv = argv;
  return RUN_ALL_TESTS();
}

TEST(Assignment1, pathThroughMaze)
{
  /* Load config file */
  auto programConfig = load_config_file(main_argc, main_argv);
  
  /* Plan path */
  Planner planner = constructPlanner(programConfig);
  planner.planPath();

  EXPECT_TRUE(planner._path_found);
  EXPECT_EQ(planner._path_node_IDs.front(), planner._entrance_nodeID);
  EXPECT_EQ(planner._path_node_IDs.back(), planner._finish_nodeID);
  if (main_argc > 1) {
    std::string jsonfilename = main_argv[1];
    if (jsonfilename == "../config/params_maze_small.json") {
      EXPECT_EQ(planner._path_node_IDs.size(), 26);
    } else if (jsonfilename == "../config/params_maze_large.json") {
      EXPECT_EQ(planner._path_node_IDs.size(), 62);
    }
  }

  bool all_path_nodes_connected = true;
  int i_prev_node;
  std::vector<int> conn_prev_node;
  for (int i_node : planner._path_node_IDs){
    if (i_node != planner._path_node_IDs.front()) {
      // check if this node is connected to the previous node
      conn_prev_node = planner._connections[i_prev_node];
      if (std::find(conn_prev_node.begin(), conn_prev_node.end(), i_node) == conn_prev_node.end()){
        all_path_nodes_connected = false;
        break;
      }
    }
    i_prev_node = i_node;
  }
  EXPECT_TRUE(all_path_nodes_connected);


  /* Simulate the robot moving along the planned path (if found) */
  bool use_debug_prints = true;
  if (planner._path_found) {
    if (use_debug_prints) {
        std::cout << "Path from entrance to finish found, visiting the following nodes: ";
        for (int node_id : planner._path_node_IDs){
            if (node_id != planner._entrance_nodeID) {
                std::cout << "-";
            }
            std::cout << node_id;
        }
        std::cout << std::endl << "I will now follow the path!" << std::endl;
    }
    simulate_path_following(planner._nodelist, planner._path_node_IDs);
  } else {
    if (use_debug_prints){
      std::cout << "No path could be found" << std::endl;
    }
  }

  // Finished, exit test
}
