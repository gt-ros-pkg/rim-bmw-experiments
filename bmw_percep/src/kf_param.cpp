#include <iostream>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>
#include <string>

/** 
    Reads in a file with 2D points, and time-diffs. Kalman
    Filters them with different params. Stores the points with
    the filtered estimates, velocities and accelerations in as many
    files as the parameters
**/

//Param being changed is jerk
#define MIN_JERK .005
#define MAX_JERK 2.
#define JERK_STEP MIN_JERK

using namespace std;

//copied from: http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
class CSVRow
{
public:
  string const& operator[](size_t index) const
  {
    return m_data[index];
  }
  size_t size() const
  {
    return m_data.size();
  }
  void readNextRow(istream& str)
  {
    string         line;
    getline(str,line);

    stringstream   lineStream(line);
    string         cell;

    m_data.clear();
    while(getline(lineStream,cell,','))
      {
	m_data.push_back(cell);
      }
  }
private:
  vector<string>    m_data;
};

istream& operator>>(istream& str,CSVRow& data)
{
  data.readNextRow(str);
  return str;
}   

//MAIN
int main(int argc, char** argv)
{

  string path="data/kf_params/normal1/";
  std::ifstream file("observations.csv");

  CSVRow row;
  while(file >> row)
    {
      std::cout << "4th Element(" << row[3] << ")\n";
    }

}
