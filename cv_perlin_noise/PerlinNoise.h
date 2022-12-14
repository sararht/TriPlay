//
//	PerlinNoise.{h|cpp} - perlin noise image generator.
//
// github:
//     https://github.com/yoggy/cv_perlin_noise
//
// license:
//     Copyright (C) 2015 yoggy <yoggy0@gmail.com>
//     Released under the MIT license
//     http://opensource.org/licenses/mit-license.php
//
// reference:
//     http://cs.nyu.edu/~perlin/noise/
//
#pragma once

#pragma warning(disable : 4819)
#include <opencv2/core.hpp>

extern cv::Mat CreatePerlinNoiseImage(const cv::Size &size, const double &scale_x = 0.05,  const double &scale_y = 0.05);

