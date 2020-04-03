#pragma once
#include <vector>
#include <memory>

struct FeatureMapInterface {
 public:
  int channels;
  int width;
  int height;
  int range;
  float* max_height_data;      // channnel 0
  float* mean_height_data;     // channnel 1
  float* count_data;           // channnel 2
  float* direction_data;       // channnel 3
  float* top_intensity_data;   // channnel 4
  float* mean_intensity_data;  // channnel 5
  float* distance_data;        // channnel 6
  float* nonempty_data;        // channnel 7
  std::vector<float> map_data;
  virtual void initializeMap(std::vector<float>& map) = 0;
  virtual void resetMap(std::vector<float>& map) = 0;
  FeatureMapInterface(const int _channels, const int _width, const int _height, const int _range);
};

struct FeatureMap : public FeatureMapInterface {
  FeatureMap(const int width, const int height, const int range);
  void initializeMap(std::vector<float>& map) override;
  void resetMap(std::vector<float>& map) override;
};

struct FeatureMapWithIntensity : public FeatureMapInterface {
  FeatureMapWithIntensity(const int width, const int height, const int range);
  void initializeMap(std::vector<float>& map) override;
  void resetMap(std::vector<float>& map) override;
};

struct FeatureMapWithConstant : public FeatureMapInterface {
  FeatureMapWithConstant(const int width, const int height, const int range);
  void initializeMap(std::vector<float>& map) override;
  void resetMap(std::vector<float>& map) override;
};

struct FeatureMapWithConstantAndIntensity : public FeatureMapInterface {
  FeatureMapWithConstantAndIntensity(const int width, const int height, const int range);
  void initializeMap(std::vector<float>& map) override;
  void resetMap(std::vector<float>& map) override;
};
