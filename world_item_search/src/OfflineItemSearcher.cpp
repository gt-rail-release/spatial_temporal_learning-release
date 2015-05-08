/*!
 * \file OfflineItemSearcher.cpp
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * Note that this node will clear the spatial world database several times. You should only run this node on a test
 * database, not your robot's persistent database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

// World Item Search
#include "world_item_search/OfflineItemSearcher.h"

// ROS
#include <ros/package.h>

// Boost
#include <boost/filesystem.hpp>

// C++ Standard Library
#include <fstream>

using namespace std;
using namespace rail::spatial_temporal_learning;

OfflineItemSearcher::OfflineItemSearcher() : worldlib::remote::Node()
{
  // location of the GeoLife files
  string geolife(ros::package::getPath("world_item_search") + "/geolife");
  private_node_.getParam("geolife", geolife);
  // open the Geolife files for model generation
  this->loadGeoLife(geolife);
  okay_ &= !geolife_.empty();
  if (!okay_)
  {
    ROS_ERROR("Unable to load any GeoLife PLT files in '%s'.", geolife.c_str());
  }

  // create the clients we need
  interactive_world_model_client_ = this->createInteractiveWorldModelClient();
  spatial_world_client_ = this->createSpatialWorldClient();

  // our default "room"
  world_.addRoom(worldlib::world::Room("default"));
  interactive_world_.addRoom(worldlib::world::Room("default"));

  // grab the interactive world models
  uint32_t task_id = worldlib::remote::InteractiveWorldModelClient::TASK_ID_PUT_AWAY_GENERAL;
  okay_ &= interactive_world_model_client_->getTaskModel(task_id, interactive_world_task_model_);
  interactive_world_task_model_.getUniqueItems(interactive_world_.getItems());
  interactive_world_task_model_.getUniqueSurfaces(interactive_world_.getRoom(0).getSurfaces());

  // initialize the objects we have in our "world"
  world_.addItem(worldlib::world::Item("spoon"));
  world_.addItem(worldlib::world::Item("fork"));
  world_.addItem(worldlib::world::Item("bowl"));
  world_.addItem(worldlib::world::Item("keys"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Coffee Table"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Sink Unit"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Dining Table with Chairs"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Dresser"));

  // attempt to connect to the spatial world database
  okay_ &= spatial_world_client_->connect();

  // set when samples are drawn
  min_lat_ = numeric_limits<double>::infinity();
  max_lat_ = -numeric_limits<double>::infinity();
  min_lng_ = numeric_limits<double>::infinity();
  max_lng_ = -numeric_limits<double>::infinity();

  if (okay_)
  {
    ROS_INFO("Offline Item Searcher Initialized");
  }
}

OfflineItemSearcher::~OfflineItemSearcher()
{
  // clean up clients
  delete interactive_world_model_client_;
  delete spatial_world_client_;
}

void OfflineItemSearcher::loadGeoLife(const std::string &directory)
{
  // use Boost to search the directory
  boost::filesystem::path path(directory.c_str());
  if (boost::filesystem::exists(path))
  {
    // the order is not defined, so we will sort a list instead
    vector<string> files;
    for (boost::filesystem::directory_iterator itr(path); itr != boost::filesystem::directory_iterator(); ++itr)
    {
      // check for a .plt file
      if (itr->path().extension().string() == ".plt")
      {
        files.push_back(itr->path().string());
      }
    }
    std::sort(files.begin(), files.end());
    for (size_t i = 0; i < files.size(); i++)
    {
      // open the file and read each line
      ifstream myfile(files[i].c_str());
      string line;
      uint32_t line_count = 0;
      while (std::getline(myfile, line))
      {
        // first 6 lines don't contain data
        if (++line_count > 6)
        {
          // split the line based on ','
          GeoLifeEntry entry;
          stringstream ss(line);
          string token;
          uint32_t token_count = 0;
          while (std::getline(ss, token, ',') && ++token_count <= 5)
          {
            if (token_count == 1)
            {
              double latitude = boost::lexical_cast<double>(token);
              entry.setLatitude(latitude);
            }
            if (token_count == 2)
            {
              double longitude = boost::lexical_cast<double>(token);
              entry.setLongitude(longitude);
            }
            else if (token_count == 5)
            {
              // date is "days since 12/30/1899" -- concert to posix time (move to 1/1/1970 then convert to seconds)
              double days = boost::lexical_cast<double>(token) - 25569;
              ros::Time time(round(days * 86400));
              entry.setTime(time);
            }
          }
          geolife_.push_back(entry);
        }
      }
    }
  }
}

void OfflineItemSearcher::run()
{
  cout << "=== Begining Simulated Item Search Experiments ===" << endl;

  // run the GeoLife experiment
  this->runGeoLifeExperiment();

  cout << "=== Simulated Item Search Experiments Finished ===" << endl;

}

void OfflineItemSearcher::runGeoLifeExperiment()
{
  cout << endl << "=== Start GeoLife Experiment ===" << endl;

  // generate random points from the first 3/4 of the data points from a uniform distribution
  int observation_pool_size = round((geolife_.size() * (3.0 / 4.0)));
  int verification_pool_size = geolife_.size() - observation_pool_size;
  boost::uniform_int<> observation_dist(0, observation_pool_size - 1);
  boost::uniform_int<> verification_dist(observation_pool_size, geolife_.size() - 1);
  boost::variate_generator<boost::mt19937, boost::uniform_int<> > observation_generator(random_, observation_dist);
  boost::variate_generator<boost::mt19937, boost::uniform_int<> > verification_generator(random_, verification_dist);

  // generate 20% of observations (will need to order)
  vector<int> rand_observations;
  int num_observations = round(0.2 * observation_pool_size);
  cout << "  Generating " << num_observations << " random indices out of " << observation_pool_size
  << " for observations... " << flush;
  while (rand_observations.size() < num_observations)
  {
    // get a random unique number
    int generated;
    do
    {
      generated = observation_generator();
    } while (std::find(rand_observations.begin(), rand_observations.end(), generated) != rand_observations.end());
    rand_observations.push_back(generated);
  }
  std::sort(rand_observations.begin(), rand_observations.end());
  cout << "done." << endl;

  // generate 20% of verification points
  vector<int> rand_verification;
  int num_verifications = round(0.2 * verification_pool_size);
  cout << "  Generating " << num_verifications << " random indices out of " << verification_pool_size
  << " for verification... " << flush;
  while (rand_verification.size() < num_verifications)
  {
    // get a random unique number
    int generated;
    do
    {
      generated = verification_generator();
    } while (std::find(rand_verification.begin(), rand_verification.end(), generated) != rand_verification.end());
    rand_verification.push_back(generated);
  }
  cout << "done." << endl;

  // determine the new min-max values
  cout << "  Finding bounding region... " << flush;
  for (size_t i = 0; i < rand_observations.size(); i++)
  {
    min_lat_ = min(min_lat_, geolife_[rand_observations[i]].getLatitude());
    max_lat_ = max(max_lat_, geolife_[rand_observations[i]].getLatitude());
    min_lng_ = min(min_lng_, geolife_[rand_observations[i]].getLongitude());
    max_lng_ = max(max_lng_, geolife_[rand_observations[i]].getLongitude());
  }
  for (size_t i = 0; i < rand_verification.size(); i++)
  {
    min_lat_ = min(min_lat_, geolife_[rand_verification[i]].getLatitude());
    max_lat_ = max(max_lat_, geolife_[rand_verification[i]].getLatitude());
    min_lng_ = min(min_lng_, geolife_[rand_verification[i]].getLongitude());
    max_lng_ = max(max_lng_, geolife_[rand_verification[i]].getLongitude());
  }
  cout << "[" << min_lat_ << ", " << max_lat_ << "] x [" << min_lng_ << ", " << max_lng_ << "]" << endl;

  // create and store the observations
  cout << "  Inserting samples into the spatial world database... " << flush;
  // clear the initial database
  spatial_world_client_->clearAllEntities();
  worldlib::world::Observation observation(worldlib::world::Item("Person"));
  // internal counter of where things where
  map<string, uint32_t> surface_counts;
  for (size_t i = 0; i < rand_observations.size(); i++)
  {
    const GeoLifeEntry &entry = geolife_[rand_observations[i]];
    const string surface_name = this->getGeoLifeSurface(entry.getLatitude(), entry.getLongitude());

    // update the counter
    surface_counts[surface_name]++;

    // only add observations when there are changes (dramatic speedup)
    if (observation.getSurface().getName() != surface_name)
    {
      // if the last surface name is empty, it means it was the first entry
      if (!observation.getSurface().getName().empty())
      {
        // add the last seen and mark it as removed
        const GeoLifeEntry &prev = geolife_[rand_observations[i - 1]];
        observation.setTime(prev.getTime());
        spatial_world_client_->addObservation(observation);
        spatial_world_client_->markObservationsAsRemoved(observation.getItem(), observation.getSurface(),
                                                         entry.getTime());
      }
      // add the new observation
      observation.getSurface().setName(surface_name);
      observation.setTime(entry.getTime());
      spatial_world_client_->addObservation(observation);
    }
  }
  cout << "done." << endl;

  // load each model
  cout << "  Loading initial persistence models... " << flush;
  // load the surface we have models for
  vector<string> surfaces;
  spatial_world_client_->getUniqueSurfaceNames(surfaces);
  // store in a map
  map<string, worldlib::model::PersistenceModel> models;
  for (int i = 0; i < surfaces.size(); i++)
  {
    const worldlib::world::Surface surface(surfaces[i]);
    models[surface.getName()] = spatial_world_client_->getPersistenceModel(observation.getItem(), surface);
  }
  cout << "done." << endl;

  // check the accuracy of the model
  cout << "  Running accuarcy test... " << endl;
  size_t model_correct = 0, most_freq_correct = 0, greedy_correct = 0, random_correct = 0;
  // get the most frequent surface
  uint32_t best_freq_count = 0;
  string most_freq_guess;
  for (size_t i = 0; i < surfaces.size(); i++)
  {
    if (surface_counts[surfaces[i]] > best_freq_count)
    {
      best_freq_count = surface_counts[surfaces[i]];
      most_freq_guess = surfaces[i];
    }
  }
  // used for random guess
  boost::uniform_int<> random_surface_dist(0, surfaces.size() - 1);
  boost::variate_generator<boost::mt19937, boost::uniform_int<> > random_surface(random_, random_surface_dist);
  for (int i = 0; i < rand_verification.size(); i++)
  {
    // get the ground truth of where the "item" is located
    const GeoLifeEntry &entry = geolife_[rand_verification[i]];
    const string ground_truth_name = this->getGeoLifeSurface(entry.getLatitude(), entry.getLongitude());

    // check for the best guess by our model
    double model_best_p = -numeric_limits<double>::infinity();
    string model_guess_name;
    for (int j = 0; j < surfaces.size(); j++)
    {
      const worldlib::model::PersistenceModel &model = models[surfaces[j]];
      double p = model.getProbabilityItemStillExists(entry.getTime());
      if (p > model_best_p)
      {
        model_guess_name = model.getSurface().getName();
        model_best_p = p;
      }
    }

    // randomly guess
    string random_guess_name = surfaces[random_surface()];

    // greedy last seen
    const GeoLifeEntry &greedy_entry = geolife_[(i == 0) ? rand_observations.back() : rand_verification[i - 1]];
    string greedy_guess_name = this->getGeoLifeSurface(greedy_entry.getLatitude(), greedy_entry.getLongitude());

    // check if we matches
    if (model_guess_name == ground_truth_name)
    {
      model_correct++;
    }
    if (most_freq_guess == ground_truth_name)
    {
      most_freq_correct++;
    }
    if (greedy_guess_name == ground_truth_name)
    {
      greedy_correct++;
    }
    if (random_guess_name == ground_truth_name)
    {
      random_correct++;
    }

    // update the observed time
    models[ground_truth_name].setLastSeen(entry.getTime());
  }

  double model_percentage = (((double) model_correct) / rand_verification.size()) * 100.0;
  cout << "         Model Rate: ";
  cout << model_percentage << "% (" << model_correct << "/" << rand_verification.size() << ")" << endl;
  double most_freq_percentage = (((double) most_freq_correct) / rand_verification.size()) * 100.0;
  cout << "    Most Freq. Rate: ";
  cout << most_freq_percentage << "% (" << most_freq_correct << "/" << rand_verification.size() << ")" << endl;
  double greedy_percentage = (((double) greedy_correct) / rand_verification.size()) * 100.0;
  cout << "        Greedy Rate: ";
  cout << greedy_percentage << "% (" << greedy_correct << "/" << rand_verification.size() << ")" << endl;
  double random_percentage = (((double) random_correct) / rand_verification.size()) * 100.0;
  cout << "        Random Rate: ";
  cout << random_percentage << "% (" << random_correct << "/" << rand_verification.size() << ")" << endl;

  cout << "=== End GeoLife Experiment ===" << endl << endl;
}

string OfflineItemSearcher::getGeoLifeSurface(const double latitude, const double longitude) const
{
  double row_size = (max_lng_ - min_lng_) / NUM_GEOLIFE_ROWS;
  double col_size = (max_lat_ - min_lat_) / NUM_GEOLIFE_COLUMNS;

  int row = (int) ((longitude - min_lng_) / row_size);
  int col = (int) ((latitude - min_lat_) / col_size);
  stringstream ss;
  ss << row << "-" << col;
  return ss.str();
}

void OfflineItemSearcher::printItemList(const vector<worldlib::world::Item> &items) const
{
  cout << "[";
  for (size_t i = 0; i < items.size(); i++)
  {
    cout << items[i].getName();
    (i < items.size() - 1) ? cout << ", " : cout << "";
  }
  cout << "]" << endl;
}

void OfflineItemSearcher::printSurfaceList(const vector<worldlib::world::Surface> &surfaces) const
{
  cout << "[";
  for (size_t i = 0; i < surfaces.size(); i++)
  {
    cout << surfaces[i].getName();
    (i < surfaces.size() - 1) ? cout << ", " : cout << "";
  }
  cout << "]" << endl;
}
