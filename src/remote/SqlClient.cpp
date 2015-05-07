/*!
 * \file SqlClient.cpp
 * \brief The main MySQL client connection.
 *
 * The worldlib SQL client can communicate with a MySQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

// worldlib
#include "worldlib/remote/SqlClient.h"

// ROS
#include <ros/ros.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::remote;

SqlClient::SqlClient(const SqlClient &client)
    : Client(client.getHost(), client.getPort()),
      user_(client.getUser()), password_(client.getPassword()), database_(client.getDatabase())
{
  connection_ = NULL;

  // check if a connection was made
  if (client.connected())
  {
    this->connect();
  }
}

SqlClient::SqlClient(const string &host, const uint16_t port, const string &user, const string &password,
    const string &database) : Client(host, port), user_(user), password_(password), database_(database)
{
  connection_ = NULL;
  connected_ = false;
}

SqlClient::~SqlClient()
{
  // check for an existing connection
  this->disconnect();
}

const string &SqlClient::getUser() const
{
  return user_;
}

const string &SqlClient::getPassword() const
{
  return password_;
}

const string &SqlClient::getDatabase() const
{
  return database_;
}

bool SqlClient::connected() const
{
  return connection_ != NULL && connected_;
}

bool SqlClient::connect()
{
  // check for an existing connection
  this->disconnect();

  // setup the client connection
  connection_ = mysql_init(NULL);
  connected_ = mysql_real_connect(connection_, this->getHost().c_str(), user_.c_str(), password_.c_str(),
                                  database_.c_str(), this->getPort(), NULL, 0);
  if (!connected_)
  {
    this->printSqlError();
  }

  return this->connected();
}

void SqlClient::disconnect()
{
  // check for an existing connection
  if (connection_ != NULL)
  {
    if (this->connected())
    {
      mysql_close(connection_);
    }
    connection_ = NULL;
    connected_ = false;
  }
}

MYSQL_RES *SqlClient::query(const string &query) const
{
  if (this->connected())
  {
    if (mysql_query(connection_, query.c_str()) == 0)
    {
      // parse and get it
      return mysql_use_result(connection_);
    } else
    {
      this->printSqlError();
    }
  } else
  {
    ROS_WARN("MySQL attempted to make a query while it was not connected.");
  }

  // something went wrong
  return NULL;
}

void SqlClient::printSqlError() const
{
  ROS_ERROR("MySQL Error: %s", mysql_error(connection_));
}
