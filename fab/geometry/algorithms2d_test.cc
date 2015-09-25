#include <vector>
using std::vector;

#include <Eigen/Dense>
using namespace Eigen;

#include <boost/smart_ptr/scoped_ptr.hpp>
#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <glog/logging.h>

#include "fab/geometry/algorithms2d.h"

using mds::algo_2d::Indexer;
using mds::algo_2d::InnerOffsetPolygon;
using mds::algo_2d::PolygonPtAligner;

TEST(PolygonPtAlignerTest, Simple) {
  vector<Vector3d> to_align, reference, expected;
  double y = 7.5;

  reference.push_back(Vector3d(-2, y, 2));
  reference.push_back(Vector3d(2, y, 2));
  reference.push_back(Vector3d(2, y, -2));
  reference.push_back(Vector3d(-2, y, -2));

  expected.push_back(Vector3d(-1, y, 1));
  expected.push_back(Vector3d(1, y, 1));
  expected.push_back(Vector3d(1, y, -1));
  expected.push_back(Vector3d(-1, y, -1));

  // Try with mirroring
  to_align.push_back(Vector3d(1, y, 1));
  to_align.push_back(Vector3d(-1, y, 1));
  to_align.push_back(Vector3d(-1, y, -1));
  to_align.push_back(Vector3d(1, y, -1));

  boost::scoped_ptr<PolygonPtAligner> aligner(
      new PolygonPtAligner(&to_align, Indexer(2, 0)));
  ASSERT_TRUE(aligner->AlignTo(reference));

  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_TRUE(aligner->aligned(i).isApprox(expected[i]))
        << aligner->aligned(i) << "\nvs. expected:\n" << expected[i];
  }
}

TEST(PolygonPtAlignerTest, GreedyTest1) {
  vector<Vector3d> to_align, reference, expected;
  double y = 7.5;

  reference.push_back(Vector3d(3, y, 3));
  reference.push_back(Vector3d(4, y, 0));
  reference.push_back(Vector3d(3, y, -3));
  reference.push_back(Vector3d(-3, y, -3));
  reference.push_back(Vector3d(-3, y, 3));

  y = -1;
  expected.push_back(Vector3d(2, y, 2));
  expected.push_back(Vector3d(2, y, 0));
  expected.push_back(Vector3d(2, y, -2));
  expected.push_back(Vector3d(-2, y, -2));
  expected.push_back(Vector3d(-2, y, 2));

  to_align.push_back(Vector3d(2, y, -2));
  to_align.push_back(Vector3d(-2, y, -2));
  to_align.push_back(Vector3d(-2, y, 2));
  to_align.push_back(Vector3d(2, y, 2));

  boost::scoped_ptr<PolygonPtAligner> aligner(
      new PolygonPtAligner(&to_align, Indexer(2, 0)));
  ASSERT_TRUE(aligner->AlignTo(reference));

   for (int i = 0; i < expected.size(); ++i) {
    EXPECT_TRUE(aligner->aligned(i).isApprox(expected[i]))
        << aligner->aligned(i) << "\nvs. expected:\n" << expected[i];
  }
}

TEST(EqualizeSegmentsTest, Simple) {
  vector<Vector3d> input, output, expected;
  double z = 5.0;

  input.push_back(Vector3d(0, 0, z));
  input.push_back(Vector3d(1, 0, z));
  input.push_back(Vector3d(1, 0.5, z));
  input.push_back(Vector3d(1, 1, z));
  input.push_back(Vector3d(0, 1, z));

  expected.push_back(Vector3d(0, 0, z));
  expected.push_back(Vector3d(0.5, 0, z));
  expected.push_back(Vector3d(1, 0, z));
  expected.push_back(Vector3d(1, 0.5, z));
  expected.push_back(Vector3d(1, 1, z));
  expected.push_back(Vector3d(0.5, 1, z));
  expected.push_back(Vector3d(0, 1, z));
  expected.push_back(Vector3d(0, 0.5, z));

  mds::algo_2d::EqualizeSegmentLengths(input, 5, &output);
  ASSERT_EQ(expected.size(), output.size());

  for (int i = 0; i < expected.size(); ++i) {
    EXPECT_TRUE(output[i].isApprox(expected[i]))
        << output[i] << "\nvs. expected:\n" << expected[i];
  }
}

TEST(EqualizeSegmentsTest, WithMinSegNum) {
  vector<Vector3d> input, output, expected;
  double z = 5.0;

  input.push_back(Vector3d(0, 0, z));
  input.push_back(Vector3d(1, 0, z));
  input.push_back(Vector3d(1, 0.5, z));
  input.push_back(Vector3d(1, 1, z));
  input.push_back(Vector3d(0, 1, z));

  expected.push_back(Vector3d(0, 0, z));
  expected.push_back(Vector3d(0.25, 0, z));
  expected.push_back(Vector3d(0.5, 0, z));
  expected.push_back(Vector3d(0.75, 0, z));
  expected.push_back(Vector3d(1, 0, z));
  expected.push_back(Vector3d(1, 0.25, z));
  expected.push_back(Vector3d(1, 0.5, z));
  expected.push_back(Vector3d(1, 0.75, z));
  expected.push_back(Vector3d(1, 1, z));
  expected.push_back(Vector3d(0.75, 1, z));
  expected.push_back(Vector3d(0.5, 1, z));
  expected.push_back(Vector3d(0.25, 1, z));
  expected.push_back(Vector3d(0, 1, z));
  expected.push_back(Vector3d(0, 0.75, z));
  expected.push_back(Vector3d(0, 0.5, z));
  expected.push_back(Vector3d(0, 0.25, z));

  mds::algo_2d::EqualizeSegmentLengths(input, 16, &output);
  ASSERT_EQ(expected.size(), output.size());

  for (int i = 0; i < output.size(); ++i) {
    EXPECT_TRUE(output[i].isApprox(expected[i]))
        << output[i] << "\nvs. expected:\n" << expected[i];
  }
}

TEST(OffsetTest, Simple) {
  vector<Vector3d> input, output, expected;
  double y = 5.5;
  Indexer ider(2, 0);

  input.push_back(Vector3d(-2, y, 2));
  input.push_back(Vector3d(2, y, 2));
  input.push_back(Vector3d(2, y, -2));
  input.push_back(Vector3d(-2, y, -2));

  double offset = 1.0;
  ASSERT_TRUE(InnerOffsetPolygon(input, ider, offset, &output));

  expected.push_back(Vector3d(-1, y, 1));
  expected.push_back(Vector3d(1, y, 1));
  expected.push_back(Vector3d(1, y, -1));
  expected.push_back(Vector3d(-1, y, -1));
  ASSERT_EQ(output.size(), expected.size());

  // Need to align the output to compare
  boost::scoped_ptr<PolygonPtAligner> aligner(
      new PolygonPtAligner(&output, ider));
  ASSERT_TRUE(aligner->AlignTo(input));

  for (int i = 0; i < output.size(); ++i) {
    EXPECT_TRUE(aligner->aligned(i).isApprox(expected[i]))
        << aligner->aligned(i) << "\nvs. expected:\n" << expected[i];
  }
}

TEST(OffsetTest, FailForSelfIntersecting) {
  vector<Vector3d> input, output;
  double y = 5.5;
  Indexer ider(2, 0);

  input.push_back(Vector3d(-2, y, 2));
  input.push_back(Vector3d(0, y, -2));
  input.push_back(Vector3d(2, y, 2));
  input.push_back(Vector3d(2, y, -2));
  input.push_back(Vector3d(0, y, 2));
  input.push_back(Vector3d(-2, y, -2));

  double offset = 1.0;
  ASSERT_FALSE(InnerOffsetPolygon(input, ider, offset, &output));
}

TEST(PointInsideTriangle, SimpleTest) {
  double y = 5.5;
  Indexer ider(2, 0);
  EXPECT_TRUE(IsPointInsideTriangle(Vector3d(0, y, 0),
                                    Vector3d(1, y, 0),
                                    Vector3d(2, y, 2),
                                    Vector3d(1.1, y, 1), ider));
  EXPECT_TRUE(IsPointInsideTriangle(Vector3d(2, y, 2),
                                    Vector3d(0, y, 0),
                                    Vector3d(1, y, 0),
                                    Vector3d(0.3, y, 0.2), ider));
  EXPECT_FALSE(IsPointInsideTriangle(Vector3d(2, y, 2),
                                     Vector3d(0, y, 0),
                                     Vector3d(1, y, 0),
                                     Vector3d(0.5, y, 2), ider));
  EXPECT_FALSE(IsPointInsideTriangle(Vector3d(-1, y, -1),
                                     Vector3d(0.5, y, -1),
                                     Vector3d(2, y, 2),
                                     Vector3d(1, y, 1.5), ider));
  EXPECT_FALSE(IsPointInsideTriangle(Vector3d(-1, y, -1),
                                     Vector3d(0.5, y, -1),
                                     Vector3d(2, y, 2),
                                     Vector3d(-1, y, 1.5), ider));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}
