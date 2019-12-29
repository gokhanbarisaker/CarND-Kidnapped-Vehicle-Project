#include <gtest/gtest.h>
#include "../src/particle_filter.h"
#include "../src/helper_functions.h"
#include <random>

TEST(ParticleFilter, ParticleFilterInit)
{
    ParticleFilter pf;

    double x = 4983.;
    double y = 5029.;
    double theta = 1.201;

    double std[3] = {0., 0., 0.};

    size_t num_particles_expected = 2;

    pf.init(x, y, theta, std, num_particles_expected, false);

    size_t num_particles_actual = pf.particles.size();

    ASSERT_EQ(num_particles_actual, num_particles_expected);

    for (size_t i = 0; i < pf.particles.size(); i++)
    {
        Particle p = pf.particles[i];

        // This doesn't cover the random distribution of particles.
        // The feature is disabled by 0 standard deviation
        ASSERT_EQ(p.x, x);
        ASSERT_EQ(p.y, y);
        ASSERT_EQ(p.theta, theta);

        ASSERT_EQ(p.weight, 1);
    }
}

TEST(ParticleFilter, ParticleFilterPredictionWithoutNoise)
{
    ParticleFilter pf;

    pf.particles.clear();

    Particle p = {
        .id = 0,
        .x = 102.,
        .y = 65.,
        .theta = (M_PI * 5. / 8.)};

    pf.particles.push_back(p);

    double std_pos[] = {0, 0, 0};

    pf.prediction(0.1, std_pos, 110, (M_PI / 8.));

    p = pf.particles[0];

    ASSERT_NEAR(p.x, 97.592, 0.001) << "Predicted locations doesn't match: " << p.x;
    ASSERT_NEAR(p.y, 75.0774, 0.001) << "Predicted locations doesn't match: " << p.y;
    ASSERT_NEAR(p.theta, 2.002, 0.001) << "Predicted locations doesn't match: " << p.theta;
}

TEST(ParticleFilter, ParticleFilterDataAssociationNone)
{
    ParticleFilter pf;

    // Assign matchin landmark inrange landmarks into map obs

    // inrange_landmark_observations
    std::vector<LandmarkObs> ilo = {};
    // map_observations
    std::vector<LandmarkObs> mo = {{.id = 4,
                                    .x = 5.0,
                                    .y = 10.0}};

    pf.dataAssociation(ilo, mo);

    int moId = mo[0].id;

    ASSERT_EQ(moId, -1) << "Data association is unable to set invalid ID!";
}

TEST(ParticleFilter, ParticleFilterDataAssociationSingle)
{
    ParticleFilter pf;

    // Assign matchin landmark inrange landmarks into map obs

    // inrange_landmark_observations
    std::vector<LandmarkObs> ilo = {{.id = 321,
                                     .x = 100.0,
                                     .y = 20.0}};
    // map_observations
    std::vector<LandmarkObs> mo = {{.id = 4,
                                    .x = 5.0,
                                    .y = 10.0}};

    pf.dataAssociation(ilo, mo);

    int moId = mo[0].id;

    ASSERT_EQ(moId, 321) << "Data association is unable to find & assign nearest neighbour ID!";
}

TEST(ParticleFilter, ParticleFilterDataAssociationMultiple)
{
    ParticleFilter pf;

    // Assign matchin landmark inrange landmarks into map obs

    // inrange_landmark_observations
    LandmarkObs lobs0 = {.id = 91,
                         .x = 100.0,
                         .y = 20.0};

    LandmarkObs lobs1 = {.id = 92,
                         .x = 99.5,
                         .y = 20.5};

    LandmarkObs lobs2 = {.id = 93,
                         .x = -10.0,
                         .y = 40.0};

    std::vector<LandmarkObs> ilo = {lobs0, lobs1, lobs2};
    // map_observations
    LandmarkObs mobs1 = {.id = 1,
                         .x = 99.9,
                         .y = 20.1};

    LandmarkObs mobs2 = {.id = 2,
                         .x = 94.0,
                         .y = 21.0};

    LandmarkObs mobs3 = {.id = 3,
                         .x = -999999.0,
                         .y = 9999999.0};

    LandmarkObs mobs4 = {.id = 4,
                         .x = -10.0,
                         .y = 40.0};
    std::vector<LandmarkObs> mo = {mobs1, mobs2, mobs3, mobs4};

    pf.dataAssociation(ilo, mo);

    ASSERT_EQ(mo[0].id, lobs0.id) << "Data association is unable to find nearest neighbour!";
    ASSERT_EQ(mo[1].id, lobs1.id) << "Data association is unable to find nearest neighbour!";
    ASSERT_EQ(mo[2].id, lobs2.id) << "Data association is unable to find nearest neighbour!";
    ASSERT_EQ(mo[3].id, lobs2.id) << "Data association is unable to find nearest neighbour!";
}

TEST(ParticleFilter, ParticleFilterUpdateWeights)
{
    ASSERT_TRUE(false) << "TODO: Implement";
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}