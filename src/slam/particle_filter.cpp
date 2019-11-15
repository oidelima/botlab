#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    // initialize all particles to match the passed in post
    // TODO: add random pertubations so that the particles are initialized to samples from a distribution
    for(int i = 0; i < kNumParticles_; i++)
    {
        posterior_[i].pose = pose;
        posterior_[i].parent_pose = pose;
        posterior_[i].weight = 1.0/kNumParticles_;
    }
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    // function resamples from the posterior distribtion to create a new one
    // in the new distribution, particles are placed according to the weights in the posterior
    
    std::vector<particle_t> prior;
    prior.resize(kNumParticles_);

    // super naive sampling with replacement
    // not computationally efficient - should ideally do some resevoir sampling method
    for(int i = 0; i < kNumParticles_; i++)
    {
        // randomVal will be a random double between 0.0 and 1.0
        double randomVal = (double)rand() / RAND_MAX;
        double curVal = 0.0;
        for(int j = 0; j < kNumParticles_; j++)
        {
            // we select the particle if the randomVal maps to this particles position on the range [0,1]
            // note: this method REQUIRES that the particle weights be normalized properly (sum to 1.0)
            if(curVal <= randomVal 
                && (curVal + posterior_[j].weight) >= randomVal)
            {
                // TODO: add random noise to this assignment
                // should treat posterior particle of expectation of distribtion
                // rather than a fixed point
                prior[i] = posterior_[i];
                break;
            }
            else
            {
                curVal += posterior_[j].weight;
            }
        }
    }
    return prior;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    // loops through the resampled particles and applies the action model to each one
    std::vector<particle_t> proposal;
    proposal.resize(kNumParticles_);
    for(int i = 0; i < kNumParticles_; i++)
    {
        proposal[i] = actionModel_.applyAction(prior[i]);
    }
    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;
    posterior.resize(kNumParticles_);
    for(int i = 0; i < kNumParticles_; i++)
    {
        
    }
    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    // final pose estimate is taken to be the MLE of the existing particles
    pose_xyt_t pose = posterior[0];
    for(int i = 1; i < kNumParticles_; i++)
    {
        if(posterior[i].weight > pose.weight)
        {
            pose = posterior[i];
        }
    }
    return pose;
}
