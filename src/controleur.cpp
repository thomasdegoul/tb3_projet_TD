#include <cmath>
#include <csignal>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <string>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>


#include <tb3_projet_TD/set_behavior.h>
#include <actionlib/server/simple_action_server.h>
#include <tb3_projet_TD/MoveAction.h>

#define FREQUENCY 10.0
#define MAX_LINEAR_SPEED 0.15
#define MAX_ANGULAR_SPEED 2.84
#define LINEAR_SPEED 0.1
#define ANGULAR_SPEED 0.1

using namespace ros;
using namespace std_msgs;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace tf;
using namespace tb3_projet_TD;
using namespace actionlib;
using namespace sensor_msgs; 



enum Mode { MODE_MANUEL, MODE_AUTONOME };

void sigintHandler(int sig);
void arreter();
void avancer(double v);
void reculer(double v);
void pivoter(double v);
void navigation_manuelle();
void aller_a_la_cible();

typedef SimpleActionServer<MoveAction> Server;

double current_cmd_lin = 0.0;
double current_cmd_ang = 0.0;
double distance_parcourue = 0.0;

Publisher pub_msg;
Publisher pub_vel;
Publisher pub_reset;

String msg;
double linear_speed = LINEAR_SPEED;
double angular_speed = ANGULAR_SPEED;
double frequence = FREQUENCY;

double distance_min = 1000.0; 

double pos_x = 0.0;
double pos_y = 0.0;
double pos_yaw = 0.0;

double target_x = 0.5;
double target_y = 0.3;

double Kp_pos = 0.5;
double Kp_angle = 0.9;

Mode mode_actuel = MODE_MANUEL;
int derniere_touche = 0;
int etat_navigation = 2;
bool obstacle_detecte = false;

float cal_delta_Theta(float _theta, float _theta0) {
  float theta, theta0, delta_theta;

  if (_theta < 0)
    theta = _theta + 2 * M_PI;
  else
    theta = _theta;

  if (_theta0 < 0)
    theta0 = _theta0 + 2 * M_PI;
  else
    theta0 = _theta0;

  delta_theta = theta - theta0;

  if (delta_theta < -M_PI) {
    delta_theta = delta_theta + 2 * M_PI;
  } else {
    if (delta_theta >= M_PI) {
      delta_theta = delta_theta - 2 * M_PI;
    }
  }
  return delta_theta;
}

void odomCallback(Odometry msg_odom) {
  static double last_x = msg_odom.pose.pose.position.x;
  static double last_y = msg_odom.pose.pose.position.y;
  
  double dx = msg_odom.pose.pose.position.x - last_x;
  double dy = msg_odom.pose.pose.position.y - last_y;
  
  if (mode_actuel == MODE_AUTONOME) {
    distance_parcourue += sqrt(dx*dx + dy*dy);
  }
  
  last_x = msg_odom.pose.pose.position.x;
  last_y = msg_odom.pose.pose.position.y;

  pos_x = msg_odom.pose.pose.position.x;
  pos_y = msg_odom.pose.pose.position.y;
  pos_yaw = getYaw(msg_odom.pose.pose.orientation);
}
void scanCallback(LaserScan msg) {
  obstacle_detecte = false;
  int num_ranges = msg.ranges.size();

  
  for (int i = 0; i < num_ranges; i++) {
    if (i < 30 || i > num_ranges - 30) {
      
      if (msg.ranges[i] > 0.01 && msg.ranges[i] < 0.30) {
        obstacle_detecte = true;
        break; 
      }
    }
  }
}
void keyCallback(Int8 msg_key) {
  derniere_touche = msg_key.data;

  if (derniere_touche == 109 || derniere_touche == 77) {
    if (mode_actuel == MODE_MANUEL) {
      mode_actuel = MODE_AUTONOME;
      etat_navigation = 0;
      ROS_INFO("passage en mode autonome");
    } else {
      mode_actuel = MODE_MANUEL;
      etat_navigation = 2;
      ROS_INFO("passage en mode manuel");
      arreter();
    }
    derniere_touche = 0;
  }
}

bool set_behavior_callback(set_behavior::Request &req,
                           set_behavior::Response &res) {

  if (req.mode == 0) {
    mode_actuel = MODE_MANUEL;
    arreter();

    res.success = true;
    ROS_INFO("Service Behavior => Succes, Passage en mode Manuel.");
  } else if (req.mode == 1) {
    mode_actuel = MODE_AUTONOME; 
    linear_speed = 0.1;
    etat_navigation = 0;

    res.success = true;
    ROS_INFO("Service Behavior => Succes, Passage en mode Auto Standard.");
  } else if (req.mode == 2) {
    mode_actuel = MODE_AUTONOME; 
    linear_speed = 0.15;
    etat_navigation = 0;

    res.success = true;
    ROS_INFO("Service Behavior => Succes, Passage en mode Auto Turbo.");
  } else {
    res.success = false;
    ROS_WARN("Service Behavior => Erreur, Mode inconnu.");
  }

  res.mode_courant = req.mode;
  return true;
}

void arreter() {
  Twist msg_vel;
  msg_vel.linear.x = 0.0;
  msg_vel.angular.z = 0.0;
  current_cmd_lin = 0.0; 
  current_cmd_ang = 0.0;
  pub_vel.publish(msg_vel);
}

void avancer(double v) {
  Twist msg_vel;
  msg_vel.linear.x = v;
  pub_vel.publish(msg_vel);
  current_cmd_lin = v; 
  current_cmd_ang = 0.0;
}

void reculer(double v) {
  Twist msg_vel;
  msg_vel.linear.x = -v;
  current_cmd_lin = -v; 
  current_cmd_ang = 0.0;
  pub_vel.publish(msg_vel);
}

void pivoter(double v_angulaire) {
  Twist msg_vel;
  msg_vel.linear.x = 0.0;
  msg_vel.angular.z = v_angulaire;
  current_cmd_lin = 0.0; 
  current_cmd_ang = v_angulaire;
  pub_vel.publish(msg_vel);
}
void executeAction(const MoveGoalConstPtr &goal, Server *as) {
  tb3_projet_TD::MoveFeedback fb;
  tb3_projet_TD::MoveResult res;

  ROS_INFO("Action recue : Navigation vers x=%f, y=%f", goal->target_x, goal->target_y);

 
  target_x = goal->target_x;
  target_y = goal->target_y;
  distance_parcourue = 0.0;
  mode_actuel = MODE_AUTONOME;
  etat_navigation = 0; 

  ros::Time start_time = ros::Time::now();
  ros::Rate loop_rate(FREQUENCY);

  
  while (ros::ok() && etat_navigation != 2) {
    
    
    if (as->isPreemptRequested() || !ros::ok()|| mode_actuel == MODE_MANUEL) {
      ROS_INFO("Action annulee par le client !");
      arreter();
      mode_actuel = MODE_MANUEL;
      as->setPreempted();
      return; 
    }

    
    aller_a_la_cible();

    
    fb.pos_x = pos_x;
    fb.pos_y = pos_y;
    fb.vitesse_lineaire = current_cmd_lin;
    fb.vitesse_angulaire = current_cmd_ang;
    fb.distance_restante = sqrt(pow(target_x - pos_x, 2) + pow(target_y - pos_y, 2));
    
    as->publishFeedback(fb);

    
    ros::spinOnce();
    loop_rate.sleep();
  }

  
  arreter();
  res.distance_totale = distance_parcourue;
  res.temps_ecoule = (ros::Time::now() - start_time).toSec();
  
  ROS_INFO("Objectif atteint ! Distance: %f m, Temps: %f s", res.distance_totale, res.temps_ecoule);
  as->setSucceeded(res);
}

void navigation_manuelle() {
  static int action_en_cours = 32;

  if (derniere_touche != 0) {
    switch (derniere_touche) {
    case 43:
      linear_speed += 0.01;
      if (linear_speed > MAX_LINEAR_SPEED)
        linear_speed = MAX_LINEAR_SPEED;
      ROS_INFO("nouvelle vitesse lineaire : %f m/s", linear_speed);
      break;
    case 45:
      linear_speed -= 0.01;
      if (linear_speed < 0.0)
        linear_speed = 0.0;
      ROS_INFO("nouvelle vitesse lineaire : %f m/s", linear_speed);
      break;
    case 42:
      angular_speed += 0.1;
      if (angular_speed > MAX_ANGULAR_SPEED)
        angular_speed = MAX_ANGULAR_SPEED;
      ROS_INFO("nouvelle vitesse angulaire : %f rad/s", angular_speed);
      break;
    case 47:
      angular_speed -= 0.1;
      if (angular_speed < 0.0)
        angular_speed = 0.0;
      ROS_INFO("nouvelle vitesse angulaire : %f rad/s", angular_speed);
      break;
    case 114: {
      ROS_INFO("reset de l'odometrie");
      Empty empty_msg;
      pub_reset.publish(empty_msg);
      break;
    }
    case 122:
      action_en_cours = 122;
      ROS_INFO("commande : avancer");
      break;
    case 115:
      action_en_cours = 115;
      ROS_INFO("commande : reculer");
      break;
    case 113:
      action_en_cours = 113;
      ROS_INFO("commande : pivoter a gauche");
      break;
    case 100:
      action_en_cours = 100;
      ROS_INFO("commande : pivoter a droite");
      break;
    case 32:
      action_en_cours = 32;
      ROS_INFO("commande : arret");
      break;
    }
    derniere_touche = 0;
  }

  switch (action_en_cours) {
  case 122:
    avancer(linear_speed);
    break;
  case 115:
    reculer(linear_speed);
    break;
  case 113:
    pivoter(angular_speed);
    break;
  case 100:
    pivoter(-angular_speed);
    break;
  case 32:
  default:
    arreter();
    break;
  }
}





void aller_a_la_cible() {
   if (obstacle_detecte) {
    arreter();
    ROS_WARN_THROTTLE(1.0, "OBSTACLE DETECTE ! En attente de degagement...");
    return;
  }
  double cible_theta, erreur_angle, distance, commande_angulaire, commande_lineaire;

  switch (etat_navigation) {
  case 0:
    cible_theta = atan2(target_y - pos_y, target_x - pos_x);
    erreur_angle = cal_delta_Theta(cible_theta, pos_yaw);

    if (fabs(erreur_angle) <= 0.05) {
      arreter();
      distance_min = 1000.0; 
      etat_navigation = 1;
      ROS_INFO("alignement vers la cible termine");
      break;
    }

    commande_angulaire = Kp_angle * erreur_angle;
    if (commande_angulaire > MAX_ANGULAR_SPEED)
      commande_angulaire = MAX_ANGULAR_SPEED;
    if (commande_angulaire < -MAX_ANGULAR_SPEED)
      commande_angulaire = -MAX_ANGULAR_SPEED;
    pivoter(commande_angulaire);
    ROS_INFO_THROTTLE(0.5, "etape 1 (pivot) - erreur d'angle : %f rad",erreur_angle);
    break;

  case 1:
    distance = sqrt(pow(target_x - pos_x, 2) + pow(target_y - pos_y, 2));

    
    if (distance <= 0.02) {
      arreter();
      etat_navigation = 2;
      ROS_INFO("etape 2 terminee : cible atteinte");
      break;
    }

    
    if (distance < distance_min) {
      distance_min = distance;
    } else if (distance > distance_min + 0.01) {
      arreter();
      etat_navigation = 0;
      ROS_INFO("cible depassee : nouvel alignement");
      break;
    }

    commande_lineaire = Kp_pos * distance;
    
    if (commande_lineaire > MAX_LINEAR_SPEED)
      commande_lineaire = MAX_LINEAR_SPEED;


    avancer(commande_lineaire);
    ROS_INFO_THROTTLE(0.5, "etape 2 (avance) - distance restante : %f m",distance);
    break;

  case 2:
  default:
    arreter();
    break;
  }
}

int main(int argc, char **argv) {

  signal(SIGINT, sigintHandler);
  ros::init(argc, argv, "controleur", ros::init_options::NoSigintHandler);

  NodeHandle nh;

  nh.param("frequence_controle", frequence, FREQUENCY);
  std::string mode_param;
  nh.param<std::string>("mode_par_defaut", mode_param, "MANUEL");

  if (mode_param == "AUTONOME") {
    mode_actuel = MODE_AUTONOME;
  } else {
    mode_actuel = MODE_MANUEL;
  }

  Subscriber sub_key = nh.subscribe("/keyboard_input", 1000, keyCallback);
  Subscriber sub_odom = nh.subscribe("/odom", 10, odomCallback);
  Subscriber sub_scan = nh.subscribe("/scan", 10, scanCallback);

  pub_msg = nh.advertise<String>("/log", 1000);
  pub_vel = nh.advertise<Twist>("/cmd_vel", 10);
  pub_reset = nh.advertise<Empty>("/reset", 10);
  ros::ServiceServer service_behavior =nh.advertiseService("/set_behavior", set_behavior_callback);
  
  Server server(nh, "moving", boost::bind(&executeAction, _1, &server), false);
  server.start();

  Rate loop_rate(frequence);

  while (ok()) {
    if (mode_actuel == MODE_MANUEL) {
      navigation_manuelle();
    }

    spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void sigintHandler(int sig) {
  ROS_INFO("fermeture du noeud controleur...");
  arreter();

  Empty empty_msg;
  pub_reset.publish(empty_msg);

  String quit_msg;
  quit_msg.data = "Arret du noeud controleur";
  pub_msg.publish(quit_msg);
  ros::spinOnce();

  ros::Duration(0.1).sleep();
  shutdown();
}
