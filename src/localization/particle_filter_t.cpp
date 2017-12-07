#include <cmath>
#include <chrono>
#include <random>
#include "localization/particle_filter_t.h"

using namespace drwn;

particle_filter_t::particle_filter_t()
{
    initialize();
}

void particle_filter_t::initialize()
{
    field_map_t::get_instance()->initialize_field();

    m_localized = false;
    float poseX = m_config.init_x;
    float poseY = m_config.init_y;
    float poseTheta = m_config.init_theta;
    int num_particles = m_config.num_particles;
    int random_particles = m_config.random_particles;

    //std::cout << poseX << " " << poseY << " " << poseTheta << " " << num_particles << " " << random_particles << std::endl;

    if (random_particles > 0) {
        float min_x, min_y, min_theta, max_x, max_y, max_theta;
        min_x = m_config.min_x;
        min_y = m_config.min_y;
        min_theta = m_config.min_theta;
        max_x = m_config.max_x;
        max_y = m_config.max_y;
        max_theta = m_config.max_theta;

        init_particles(min_x, max_x, min_y, max_y, min_theta, max_theta, num_particles);
    } else {
        init_particles(drwn::pose2d_t(poseX, poseY, poseTheta), num_particles);
    }
}

void particle_filter_t::predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise)
{
    assert(!m_particles.empty());

    for (auto& particle : m_particles) {
        particle.pose = odometry_sample(particle.pose, command, noise);
    }
}

std::tuple<field_map_t::line_type_t, point2d_t> particle_filter_t::calc_expected_measurement(float rx, float ry, float rtheta, float measured_range, float measured_bearing)
{
    // Predicted measurement
    // Cast a line from robot position with same bearing
    float epx, epy;
    // Need to normalize angles everywhere
    pose2d_t normalizer(0.0, 0.0, measured_bearing + rtheta);

    epx = rx + field_map_t::MAX_DIST * std::cos(normalizer.get_theta());
    epy = ry + field_map_t::MAX_DIST * std::sin(normalizer.get_theta());

    // Accept intersections with distance from robot to intersection point greater than minDist 
    return field_map_t::get_instance()->intersect_with_field(
            line_t(rx, ry, epx, epy), measured_range * 0.7
    );
}

void particle_filter_t::correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise)
{
    assert(!m_particles.empty());

    if (measurements.size() < 1) {
        if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: No measurements to process";
        return;
    } else {
        if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Received " << measurements.size() << " measurements";
    }

    float weight_normalizer = 0.0f;
    for (auto& particle : m_particles) {
//        if (measurements.size() < 1) {
//            weight_normalizer = 1.0f;
//            break;
//        }

        float rx, ry, rtheta, vrange, vbearing;
        rx = particle.pose.get_x(); ry = particle.pose.get_y(); rtheta = particle.pose.get_theta();

        // Construct noise matrix
        vrange = noise(0); vbearing = noise(1);
        Eigen::MatrixXf Qt(4, 4);
        Qt << vrange, 0.0f, 0.0f, 0.0f,
              0.0f, vbearing, 0.0f, 0.0f,
              0.0f, 0.0f, vrange, 0.0f,
              0.0f, 0.0f, 0.0f, vbearing;

        pose2d_t normalizer;
        float range1, bearing1, range2, bearing2;
        float dx1, dy1, dx2, dy2;

        float new_weight = particle.weight;
        for (const auto& reading : measurements) {
            // Received measurement range bearing to 2 line points
            // Format: range1, bearing1, range2, bearing2
            range1 = reading(0); bearing1 = reading(1);
            range2 = reading(2); bearing2 = reading(3);

            // Predicted measurments
            auto test_line1 = calc_expected_measurement(rx, ry, rtheta, range1, bearing1);
            field_map_t::line_type_t ret_type1 = std::get<0>(test_line1);
            point2d_t isec_point1 = std::get<1>(test_line1);

            auto test_line2 = calc_expected_measurement(rx, ry, rtheta, range2, bearing2);
            field_map_t::line_type_t ret_type2 = std::get<0>(test_line2);
            point2d_t isec_point2 = std::get<1>(test_line2);

            if (ret_type1 == field_map_t::line_type_t::NONE || ret_type2 == field_map_t::line_type_t::NONE) {
                continue;
            }

            // Computing measurement difference (for weight calculation)
            dx1 = isec_point1.X - rx;
            dy1 = isec_point1.Y - ry;
            float expected_range1 = std::sqrt(dx1*dx1 + dy1*dy1);
            if (expected_range1 > field_map_t::MAX_DIST) {
                continue;
            }
            normalizer.set_theta(std::atan2(dy1, dx1) - rtheta);
            float expected_bearing1 = normalizer.get_theta();

            dx2 = isec_point2.X - rx;
            dy2 = isec_point2.Y - ry;
            float expected_range2 = std::sqrt(dx2*dx2 + dy2*dy2);
            if (expected_range2 > field_map_t::MAX_DIST) {
                continue;
            }
            normalizer.set_theta(std::atan2(dy2, dx2) - rtheta);
            float expected_bearing2 = normalizer.get_theta();

            // normalize angles (again) and make diff matrix
            float bdiff1, bdiff2;
            normalizer.set_theta(expected_bearing1 - bearing1);
            bdiff1 = normalizer.get_theta();
            normalizer.set_theta(expected_bearing2 - bearing2);
            bdiff2 = normalizer.get_theta();
            Eigen::Vector4f diff = {
                expected_range1 - range1,
                bdiff1,
                expected_range2 - range2,
                bdiff2,
            };

            // Calculate new particle weight
            float det = (2.0f * M_PI * Qt).determinant();
            float temp = (diff.transpose() * Qt.inverse() * diff)(0);
            new_weight *= det * std::exp((-1.0f / 2.0f) * temp);

            // Penalize the particle if isec points are not on the same line
            if (ret_type1 != ret_type2) {
                new_weight = new_weight * 0.1f;
            }


            if (std::isnan(new_weight)) {
                new_weight = 0.0f;
            }
        }

        weight_normalizer = weight_normalizer + new_weight;
        particle.weight = new_weight;
    }

    if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Done processing measurements";

    // While debugging
    //
    if (std::fabs(weight_normalizer) < 0.00001) {
        if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Normalizer is close to zero";
        weight_normalizer = 1.0f;
    }

    // Particle weight normalization
    float highestWeight = 0.0f;
    std::size_t highestWeightIndex = 0;
    for (std::size_t index = 0; index < m_particles.size(); index++) {
        particle_t& particle = m_particles[index];

        if (std::fabs(weight_normalizer) < 0.00001 || std::isnan(weight_normalizer) || std::isnan(particle.weight)) {
            // Reset all weights
            particle.weight = 1.0f / m_particles.size();

        } else {
            particle.weight = particle.weight / weight_normalizer;

        }

        if (particle.weight > highestWeight) {
            highestWeight = particle.weight;
            highestWeightIndex = index;
        }
    }
    m_topParticleIndex = highestWeightIndex;
}

void particle_filter_t::resample()
{
    low_variance_resampling();
    calc_pose_mean_cov();
    check_if_localized();
}

void particle_filter_t::low_variance_resampling()
{
    if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Start resampling";

    std::vector<particle_t> new_particles;

    float Jinv = 1.0f / m_particles.size();

    std::default_random_engine generator;
    std::uniform_real_distribution<float> dist(0.0, Jinv);
    float r = dist(generator);

    float c = m_particles[0].weight;
    int i = 0;
    particle_t new_particle;
    for (int j = 0; j < m_particles.size(); j++) {
       float U = r + (j-1) * Jinv;
       while (U > c) {
           i++;
           c += m_particles[i].weight;
       }
       new_particle.weight = 1.0f / m_particles.size();
       new_particle.pose = m_particles[i].pose;
       new_particles.push_back(new_particle);
    }

    if (new_particles.size() != m_particles.size()) {
        throw std::length_error("PARTICLE_FILTER: Length error in low_variance_resampling");
    }

    m_particles.swap(new_particles);

    if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Finish resampling";
}

Eigen::Vector4f particle_filter_t::get_line_range_bearing(pose2d_t robotPose, float x1, float y1, float x2, float y2)
{
    float rx, ry, rtheta;

    rx = robotPose.get_x();
    ry = robotPose.get_y();
    rtheta = robotPose.get_theta();

    float gx1, gy1, gx2, gy2;
    gx1 = (x1-rx)*std::cos(rtheta) + (y1-ry)*std::sin(rtheta);
    gy1 = -(x1-rx)*std::sin(rtheta) + (y1-ry)*std::cos(rtheta);
    gx2 = (x2-rx)*std::cos(rtheta) + (y2-ry)*std::sin(rtheta);
    gy2 = -(x2-rx)*std::sin(rtheta) + (y2-ry)*std::cos(rtheta);

    // Convert to range-bearing
    // format is (id, range, bearing)
    // id is unknown, so it's -1
    float dx1, dy1, dx2, dy2;
    float range1, range2, bearing1, bearing2;
    dx1 = gx1 - rx;
    dy1 = gy1 - ry;
    drwn::pose2d_t normalizer(0.0, 0.0, std::atan2(dy1, dx1) - rtheta);
    range1 = std::sqrt(dx1*dx1 + dy1*dy1);
    bearing1 = normalizer.get_theta();

    dx2 = gx2 - rx;
    dy2 = gy2 - ry;
    normalizer.set_theta(std::atan2(dy2, dx2) - rtheta);

    range2 = std::sqrt(dx2*dx2 + dy2*dy2);
    bearing2 = normalizer.get_theta();

    Eigen::Vector4f lineRangeBearing = {
            range1,
            bearing1,
            range2,
            bearing2
    };

    return lineRangeBearing;
}

void particle_filter_t::calc_pose_mean_cov()
{
    pose2d_t meanAccum;
    pose2d_t angleNormalizer;

    float pw, px, py, ptheta;
    float avg_x = 0.0f, avg_y = 0.0f;
    pw = 1.0f / m_particles.size();
    for (const auto& particle : m_particles) {
        px = particle.pose.get_x();
        py = particle.pose.get_y();
        ptheta = particle.pose.get_theta();

        avg_x += pw * std::cos(ptheta);
        avg_y += pw * std::sin(ptheta);

        meanAccum.set_x(meanAccum.get_x() + px);
        meanAccum.set_y(meanAccum.get_y() + py);
    }
    meanAccum.set_x(pw * meanAccum.get_x());
    meanAccum.set_y(pw * meanAccum.get_y());
    //meanAccum.set_theta(pw * meanAccum.get_theta());

    if (meanAccum.is_nan()) {
        if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Calculated mean_pose is NaN. Ignoring...";
        m_poseMean = this->get_top_particle().pose;
        return;
    }

    angleNormalizer.set_theta(std::atan2(avg_y, avg_x));
    meanAccum.set_theta(angleNormalizer.get_theta());

    pose2d_t devAccum;
    float mx, my, mtheta;
    mx = meanAccum.get_x();
    my = meanAccum.get_y();
    mtheta = meanAccum.get_theta();
    pw = 1.0f / m_particles.size();
    for (const auto& particle : m_particles) {
        px = particle.pose.get_x();
        py = particle.pose.get_y();
        ptheta = particle.pose.get_theta();

        // for correct stddev calculation
        // square root of mean of (xi-x.mean)**2 for all i
        angleNormalizer.set_theta(ptheta-mtheta);
        devAccum += pose2d_t(std::pow(px-mx, 2), std::pow(py-my, 2), std::pow(angleNormalizer.get_theta(), 2));
    }
    devAccum.set_x(pw * devAccum.get_x());
    devAccum.set_y(pw * devAccum.get_y());
    devAccum.set_theta(pw * devAccum.get_theta());
    // std::sqrt(mean((xi - x.mean())**2))
    devAccum.set_x(std::sqrt(devAccum.get_x()));
    devAccum.set_y(std::sqrt(devAccum.get_y()));
    devAccum.set_theta(std::sqrt(devAccum.get_theta()));

    if (devAccum.is_nan()) {
        if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: Calculated pose_std_dev is NAN. Ignoring...";
        m_poseDev.set_x(this->get_loc_threshold_x() * 2.0f);
        m_poseDev.set_y(this->get_loc_threshold_y() * 2.0f);
        m_poseDev.set_theta(this->get_loc_threshold_theta() * 2.0f);
        return;
    }

    m_poseMean = meanAccum;
    m_poseDev = devAccum;
}


void particle_filter_t::init_particles(const pose2d_t& pose, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    m_topParticleIndex = 0;
    m_localized = false;

    m_particles.resize(num_particles);
    for (auto& particle : m_particles) {
        particle.pose = pose;
        particle.weight = defaultWeight;
    }

    calc_pose_mean_cov();
}

void particle_filter_t::init_particles(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    m_topParticleIndex = 0;
    m_localized = false;

    m_particles.resize(num_particles);

    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> dist_x(min_x, max_x);
    std::uniform_real_distribution<> dist_y(min_y, max_y);
    std::uniform_real_distribution<> dist_theta(min_theta, max_theta);

    for (auto& particle : m_particles) {
        particle.pose = pose2d_t(dist_x(generator), dist_y(generator), dist_theta(generator));
        particle.weight = defaultWeight;
    }

    calc_pose_mean_cov();
}

float particle_filter_t::sample_normal_distribution(float variance)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<float> normal(0.0f, variance);

    return normal(generator);
}

pose2d_t particle_filter_t::odometry_sample(pose2d_t pose, Eigen::Vector3f command, Eigen::Vector3f noise)
{
    float rot1, trans, rot2;
    rot1 = command(0); trans = command(1); rot2 = command(2);

    float r1_noise, t_noise, r2_noise;
    r1_noise = noise(0); t_noise = noise(1); r2_noise = noise(2);

    float theta_old = pose.get_theta();

    float rot1_h, trans_h, rot2_h;
    rot1_h = rot1 - sample_normal_distribution(r1_noise);
    trans_h = trans - sample_normal_distribution(t_noise);
    rot2_h = rot2 - sample_normal_distribution(r2_noise);

    pose2d_t normalizer;
    normalizer.set_theta(theta_old + rot1_h);
    float normalized = normalizer.get_theta();

    pose2d_t newPose;
    newPose.set_x(trans_h * std::cos(normalized));
    newPose.set_y(trans_h * std::sin(normalized));
    newPose.set_theta(rot1_h + rot2_h);

    return pose + newPose;
}

Eigen::Vector3f particle_filter_t::get_odometry_command(pose2d_t prevPose, pose2d_t currPose)
{
    float rot1, trans, rot2;
    float dx, dy;
    dx = currPose.get_x() - prevPose.get_x();
    dy = currPose.get_y() - prevPose.get_y();

    trans = std::sqrt(dx*dx + dy*dy);
    pose2d_t normalizer(0.0f, 0.0f, std::atan2(dy, dx) - prevPose.get_theta());
    rot1 = normalizer.get_theta();
    normalizer.set_theta(currPose.get_theta() - prevPose.get_theta() - rot1);
    rot2 = normalizer.get_theta();

    return Eigen::Vector3f {
        rot1,
        trans,
        rot2
    };
}

void particle_filter_t::check_if_localized() {
    auto const& pose_mean = this->get_pose_mean();
    auto const& pose_deviation = this->get_pose_std_dev();

    float x_dev = pose_deviation.get_x();
    float y_dev = pose_deviation.get_y();
    float theta_dev = pose_deviation.get_theta();

    m_localized = x_dev <= this->get_loc_threshold_x() &&
                  y_dev <= this->get_loc_threshold_y() &&
                  theta_dev <= this->get_loc_threshold_theta();
}

bool particle_filter_t::is_localized() const {
    return m_localized;
}

bool particle_filter_t::is_debug_enabled() const {
    return m_debug;
}

void particle_filter_t::enable_debug(bool debug) {
    m_debug = debug;
}

void particle_filter_t::reset_pose(const pose2d_t& pose) {
    if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: resetting particles to pose = " << pose;
    init_particles(pose, m_particles.size());
}

void particle_filter_t::reset_pose(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta)
{
    if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: resetting particles to area";
    init_particles(min_x, max_x, min_y, max_y, min_theta, max_theta, m_particles.size());
}

void particle_filter_t::reset_pose_to_field()
{
    if (m_debug) LOG_DEBUG << "PARTICLE_FILTER: resetting particles to field area (set by config)";
    init_particles(m_config.min_x, m_config.max_x, m_config.min_y, m_config.max_y, m_config.min_theta, m_config.max_theta, m_particles.size());
}
