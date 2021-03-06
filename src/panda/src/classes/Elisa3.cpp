//
// Created by omdegroot on 06-08-19.
//
/*
 * Elisa3.cpp
 *
 * Model for feedback linearised Elisa3 with passivity-based control
 *
 * Measured are: x, y, theta (angle versus world frame)
 *               ddx, ddy
 *
 * This file uses x, y, theta to feedback linearise the system around the "hand point"
 * h = [x;y] + L[cos(theta); sin(theta)]
 * The dynamics around this point are a point mass
 * Since speed control is the only possibility we use
 * dq[k+1] = dq[k] + tau[k]*Ts (note that mass was compensated in feedback linearisation)
 * Then dq[k+1] is the speed target and is converted to wheel speeds
 *
 * Outputs are omega_r, omega_l
 *
 *
 */
#include "Elisa3.h"

Elisa3::Elisa3()
        :System(2, 2, 2, "elisa_3")
{
    PROFILE_SCOPE("Elisa3 init");

    logMsg("Elisa3", "Initialising...", 2);

    // Retrieve parameters
    Ts = 1.0/((double)(cmm->agent->getSamplingRate()));

    ros::NodeHandle nh;

    cmm->agent->retrieveParameter("address", address); // Retrieve the hand distance
    cmm->agent->retrieveParameter("L", L); // Retrieve the hand distance
    cmm->agent->retrieveParameter("color/intensity", intensity); // Retrieve the hand distance
    if(intensity > 1.0 || intensity < 0.0){
        throw ParameterException("Color Intensity of Elisa3 larger than 1.0 (Value should be between 0.0 and 1.0)!");
    }

    color_set = false;

    test_pub.init(cmm->agent->getPrivateNh(), "q", 1);

    // And these are the actual system states
    actual_q = Eigen::VectorXd::Zero(3);
    actual_dq = Eigen::VectorXd::Zero(3);

    Eigen::VectorXd h(2);
    h(0, 0) = actual_q(0) + L*std::cos(actual_q(2));
    h(1, 0) = actual_q(1) + L*std::sin(actual_q(2));
    
    // Initialise all state related values
    // These are the feedback linearised states
    setState(h, Eigen::VectorXd::Zero(n), h);

    // Initialise the transformation from velocities to wheel velocities
    initMatrices();

    ros::ServiceClient connect_client = nh.serviceClient<panda::registerElisa3>("/registerElisa3");

    panda::registerElisa3 srv;
    srv.request.address = address;

    if(!connect_client.call(srv)){
        throw RegisteringException("Could not register this Elisa3!");
    }

    color_client = nh.serviceClient<panda::colorElisa3>("/colorElisa3");
    cmm->agent->retrieveParameter("color/type", color_name);   

    /** @todo Fix! Elisastation hasn started communication when this is executed...*/
    
    move_pub = nh.advertise<panda::Move>("elisa3_" + std::to_string(address) + "_move", 20);
    
    readout_sub = nh.subscribe<panda::Readout>("elisa3_" + std::to_string(address) + "_readout",
                     20, &Elisa3::readSensors, this);

    cmm->performHandshake();

    logMsg("Elisa3", "Done!", 2);
}

Elisa3::~Elisa3() {
}

/// Convert the control input to the system inputs and actuate
bool Elisa3::sendInput(const Eigen::VectorXd & tau){
    
    PROFILE_FUNCTION();

    // System needs to be enabled, otherwise the system gets stuck waiting on server calls to each other...
    // Solution is probably asynchronic spinners.
    if(is_enabled && !color_set){
        setColor(color_name);
        color_set = true;
    }
    
    data_received = false;
    
    // Apply feedback linearisation around the hand point (output = F, tau)
    Eigen::VectorXd output = feedbackLinearisation(tau);

    // Artificially implement torque control instead of speed control (output = v, omega (m/s, rad/s))
    output = actual_dq + (F*output)*Ts;
    actual_dq = output;

    // Convert to wheel velocities
    output = wheel_matrix*output;

    // Convert them to integer values
    signed int wheel_right = std::floor(output(0)*1000.0/5.0); // maybe just round up?
    signed int wheel_left = std::floor(output(1)*1000.0/5.0);

    unsigned min_speed = 0;
    if(std::abs(wheel_left) <= min_speed && std::abs(wheel_right) <= min_speed){
        wheel_left = 0;
        wheel_right = 0;
    }

    // Saturate the speed to the max speed (and scale if necessary)
    saturateSpeed(wheel_right, wheel_left);

    // Apply the speeds
    move_msg.address = address;
    move_msg.wheel_right = wheel_right;
    move_msg.wheel_left = wheel_left;
    move_pub.publish(move_msg);
    
    return true;
}


void Elisa3::readSensors(const panda::Readout::ConstPtr & msg){

    /** @x towards window
     * @y towards panda
     * @theta 0.0 when in x direction*/
    /// Retrieve the accelerations ( correct compared with gtronic )
//    Eigen::VectorXd a(2); //SWAP X AND Y HERE!
//    a(0,0) = -getAccY(address);// / 64.0 * 9.81;
//    a(1,0) = getAccX(address);// / 64.0 * 9.81;
//    a *= (9.81/64.0);

    PROFILE_FUNCTION();

    Eigen::VectorXd q(3);
    q(0) = msg->x; // Inverted in the convention
    q(1) = msg->y;
    q(2) = msg->theta; // Is inverted in direction
    actual_q = q;

    /// Convert to hand position
    Eigen::VectorXd h(2);
    h(0, 0) = actual_q(0) + L*std::cos(actual_q(2));
    h(1, 0) = actual_q(1) + L*std::sin(actual_q(2));

    /// Find hand velocities for control
    Eigen::VectorXd dh(2);
    Eigen::MatrixXd A(2, 2);
    A << std::cos(actual_q(2)), -L*std::sin(actual_q(2)),
         std::sin(actual_q(2)), L*std::cos(actual_q(2));
    dh = A * actual_dq;
    
    // Set the public state
    this->setState(h, dh, h);

    data_received = true;
}

/// Check if program can proceed
void Elisa3::checkSafety(){

    return;
}

/** Dynamical Matrices and Vectors*/
Eigen::MatrixXd& Elisa3::M(){

    m_m = Eigen::MatrixXd::Identity(n, n);
    return m_m;
}

Eigen::VectorXd& Elisa3::dVdq(){

    dvdq = Eigen::VectorXd::Zero(n);
    return dvdq;
}

Eigen::MatrixXd& Elisa3::Psi(){
    psi = this->selectPsi(Eigen::MatrixXd::Identity(n, n));
    return psi;
}

Eigen::MatrixXd& Elisa3::dM(){
    dm = Eigen::MatrixXd::Zero(n, n);
    return dm;
}

Eigen::MatrixXd& Elisa3::dPsi(){
    dpsi = this->selectPsi(Eigen::MatrixXd::Zero(n, n));
    return dpsi;
}

Eigen::MatrixXd& Elisa3::dMinv()
{
    dminv = Eigen::MatrixXd::Zero(n, n);
    return dminv;
}

// Apply feedback linearisation
Eigen::VectorXd Elisa3::feedbackLinearisation(const Eigen::VectorXd& tau){
    PROFILE_FUNCTION();

    // Define the transformation (todo Optimise)
    Eigen::MatrixXd A(n, n);
    Eigen::VectorXd dA(n);
    A << (1.0/m)*std::cos(actual_q(2)), -(L/J)*std::sin(actual_q(2)),
         (1.0/m)*std::sin(actual_q(2)), (L/J)*std::cos(actual_q(2));

    dA << -actual_dq(0)*actual_dq(1)*std::sin(actual_q(2)) - L*actual_dq(1)*actual_dq(1)*std::cos(actual_q(2)),
            actual_dq(0)*actual_dq(1)*std::cos(actual_q(2)) - L*actual_dq(1)*actual_dq(1)*std::sin(actual_q(2));

    // Apply the feedback linearisation (see https://pdfs.semanticscholar.org/4f02/6f6aac864f5bc07b0f065ec2036046f3d306.pdf)
    return A.inverse()*(tau - dA);
}

/// Convert the vector [u, w] into [v_r, v_l]
Eigen::VectorXd Elisa3::velocityToWheelSpeed(const Eigen::VectorXd& vel){
    PROFILE_FUNCTION();

    // Gives [Vr, Vl]'
    return wheel_matrix*vel;
}

/// Saturate the speed to the set limit
void Elisa3::saturateSpeed(signed int & v_r, signed int & v_l) {

    if(!std::isfinite(v_r)){
            throw OperationalException("[Elisa3]: Right wheel speed (v_r) is not finite (NaN or inf)!");
    }
    
    if(!std::isfinite(v_l)){
        throw OperationalException("[Elisa3]: Left wheel speed (v_l) is not finite (NaN or inf)!");
    }
    
    // Find the maximum speed between the wheels
    int max_cur_speed = std::max(std::abs(v_r), std::abs(v_l));

    // If the speed has passed the limit
    if (max_cur_speed > max_speed) {
        logMsg("Elisa3", "Speed limit applied to Elisa3", 1);

        // Scale down the wheel speeds
        v_r = static_cast<signed int>(double(v_r) * double(max_speed) / double(max_cur_speed));
        v_l = static_cast<signed int>(double(v_l) * double(max_speed) / double(max_cur_speed));
    }
}

/// Initialise matrices
void Elisa3::initMatrices(){
    
    // The wheel transformation matrix
    wheel_matrix = Eigen::MatrixXd::Zero(n, n);
    wheel_matrix << 0.5, 0.5, 1.0/l, -1.0/l;
    wheel_matrix = wheel_matrix.inverse();

    // The input matrix
    F = Eigen::MatrixXd::Zero(2, 2);
    F << 1.0/m, 0.0, 0.0, 1.0/J;
}

void Elisa3::setColor(const std::string& color){
    
    if(color == "blue"){
        setColor(COLOR_BLUE);
    }else if(color == "green"){
        setColor(COLOR_GREEN);
    }else if(color == "red"){
        setColor(COLOR_RED);
    }else if(color == "orange"){
        setColor(COLOR_ORANGE);
    }else if(color == "cyan"){
        setColor(COLOR_CYAN);
    }else if(color == "pink"){
        setColor(COLOR_PINK);
    }
}

void Elisa3::setColor(int color_type){
    panda::colorElisa3 srv_color;
    srv_color.request.address = address;
    srv_color.request.color_type = color_type;
    srv_color.request.red = 0; srv_color.request.green = 0; srv_color.request.blue = 0;
    srv_color.request.intensity = intensity;

    color_client.call(srv_color);
    
}

void Elisa3::setColor(int r, int g, int b){
    
    panda::colorElisa3 srv_color;
    srv_color.request.address = address;
    srv_color.request.color_type = 0;
    srv_color.request.red = r; srv_color.request.green = g; srv_color.request.blue = b;
    srv_color.request.intensity = intensity;
    
    color_client.call(srv_color);
}

bool Elisa3::dataReady(){
    return data_received;
}