#include "SamplerPole.hpp"
#include <boost/random.hpp>
#include <common/Utils/YcLogger.h>
#include <ctime>

namespace{
  Utils::LoggerPtr s_logger(Utils::getLogger("uascode.SamplerPole.Yclogger"));
}

namespace UserTypes{

   SamplerPole::~SamplerPole(){}

   void SamplerPole::SetParams(UserStructs::PlaneStateSim& st,
                               UserStructs::MissionSimPt& goal_wp,
                               double _sig_ga)
   {
      double x_root= st.x;
      double y_root= st.y;
      double z_root= st.z;
      double x_goal= goal_wp.x;
      double y_goal= goal_wp.y;
      double z_goal= goal_wp.alt;

      //check start and goal position
      UASLOG(s_logger,LL_DEBUG,"root: "
             <<x_root<<" "
             <<y_root<<" "
             <<z_root);
      
      UASLOG(s_logger,LL_DEBUG,"goal: "
             <<x_goal<<" "
             <<y_goal<<" "
             <<z_goal);
      //
      double Dx= x_goal-x_root;
      double Dy= y_goal-y_root;
      double Dz= z_goal-z_root;
      double theta0= atan2(Dy,Dx);
     
      double r0,gamma0 =0.;

      if(sample_method ==0)
      { //here r0 is different
        r0= sqrt(Dx*Dx+Dy*Dy+Dz*Dz);
      }
      else if(sample_method ==1)
      {
        r0= sqrt(Dx*Dx+Dy*Dy);
      }
      else {;}

      x0 = x_root;
      y0 = y_root;
      z0 = z_root;
      this->r0 = r0;

      sigma_r= 0.5*r0;
      std::cout << "sigma_r:" << sigma_r << '\n';
      this->theta0 = theta0;
      ga0 = gamma0;
      sigma_ga= _sig_ga;
   }

   void SamplerPole::SetParams2(double x_start, double y_start, double z_start, UserStructs::MissionSimPt &goal_wp, double _sig_ga)
   {
       double x_goal= goal_wp.x;
       double y_goal= goal_wp.y;
       double z_goal= goal_wp.alt;

       //check start and goal position
       UASLOG(s_logger,LL_DEBUG,"sample root: "
              <<x_start<<" "
              <<y_start<<" "
              <<z_start);

       UASLOG(s_logger,LL_DEBUG,"sample goal: "
              <<x_goal<<" "
              <<y_goal<<" "
              <<z_goal);
       double dis_them= sqrt(pow(x_start-x_goal,2)+pow(y_start-y_goal,2)+pow(z_start-z_goal,2));
       UASLOG(s_logger,LL_DEBUG,"distance between them:"<< dis_them);
       //
       double Dx= x_goal-x_start;
       double Dy= y_goal-y_start;
       double Dz= z_goal-z_start;
       double theta0= atan2(Dy,Dx);

       double r0,gamma0 =0.;

       if(sample_method ==0)
       { //here r0 is different
         r0= sqrt(Dx*Dx+Dy*Dy+Dz*Dz);
         /*
         if(r0> 800)
             r0= 800;
             */
       }
       else if(sample_method ==1)
       {
         r0= sqrt(Dx*Dx+Dy*Dy);
       }
       else {;}

       x0 = x_start;
       y0 = y_start;
       z0 = z_start;
       UASLOG(s_logger,LL_DEBUG,"x0:"<< x0 <<" "
              "y0:"<< y0 << " "
              "z0:"<< z0);

       this->r0 = r0;
       sigma_r= 0.5*r0;
       std::cout << "sigma_r:" << sigma_r << '\n';
       this->theta0 = theta0;
       ga0 = gamma0;
       sigma_ga= _sig_ga;
   }

   void SamplerPole::GetSample(double& x_a,double& y_a,double& z_a,
			UserStructs::PlaneStateSim& st,
			UserStructs::MissionSimPt& goal_wp)
  {
      boost::mt19937 generator;
      static unsigned int seed = 0;
      generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
       
      boost::normal_distribution<> r_distribution(r0,sigma_r);
      boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > r_nor(generator, r_distribution);  
      double r= r_nor();
      
      UASLOG(s_logger,LL_DEBUG,"sample theta0:"<< theta0*180./M_PI);
      boost::uniform_real<> the_uniform(theta0-M_PI/2.0, theta0+M_PI/2.0);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > the_nor(generator, the_uniform);
      double theta= the_nor();
      if(sample_method == 0)
      {
          boost::normal_distribution<> ga_distribution(ga0, sigma_ga);
          boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > ga_nor(generator, ga_distribution);
          double ga= ga_nor();
          /*
          UASLOG(s_logger,LL_DEBUG,
                 "r: "<< r
                 << " theta: "<< theta*180./M_PI
                 << " ga: "<< ga*180./M_PI
                 ); */

          x_a= x0+ r*cos(theta)*cos(ga);
          y_a= y0+ r*sin(theta)*cos(ga);
          z_a= z0+ r*sin(ga);
      }
      else if(sample_method==1)
      {
          x_a= x0+ r*cos(theta);
          y_a= y0+ r*sin(theta);
          double d_x= x0- x_a;
          double d_y= y0- y_a;
          //get normal vector
          double x_r= st.x;
          double y_r= st.y;
          double z_r= st.z;
          double x_g= goal_wp.x;
          double y_g= goal_wp.y;
          double z_g= goal_wp.alt;

          double n_x= -1.*(x_g-x_r)*(z_g-z_r);
          double n_y= -1.*(y_g-y_r)*(z_g-z_r);
          double n_z= pow(x_g-x_r,2)+pow(y_g-y_r,2);
          assert(n_z > 0 );

          z_a= z0+ 1./n_z*( n_x*d_x+n_y*d_y );
      }

   }//GetSample ends

   void SamplerPole::GetSample2(double& x_a, double& y_a, double& z_a, double x_start, double y_start, double z_start, UserStructs::MissionSimPt &goal_wp)
   {
       //UASLOG(s_logger,LL_DEBUG,"try to sample");
       boost::mt19937 generator;
       static unsigned int seed = 0;
       generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));

       //std::cout<<"sigma_r= "<< sigma_r <<std::endl;
       boost::normal_distribution<> r_distribution(r0,sigma_r);
       boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > r_nor(generator, r_distribution);
       double r= r_nor();

       //UASLOG(s_logger,LL_DEBUG,"sample theta0:"<< theta0*180./M_PI);
       double alpha= M_PI;
       boost::uniform_real<> the_uniform(theta0 - alpha, theta0 + alpha);
       boost::variate_generator<boost::mt19937&,boost::uniform_real<> > the_nor(generator, the_uniform);
       double theta= the_nor();
       if(sample_method == 0)
       {
           boost::normal_distribution<> ga_distribution(ga0, sigma_ga);
           boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > ga_nor(generator, ga_distribution);
           double ga= ga_nor();
           x_a= x0+ r*cos(theta)*cos(ga);
           y_a= y0+ r*sin(theta)*cos(ga);
           z_a= z0;
       }
       else if(sample_method==1)
       {
           x_a= x0+ r*cos(theta);
           y_a= y0+ r*sin(theta);
           double d_x= x0- x_a;
           double d_y= y0- y_a;
           //get normal vector
           double x_r= x_start;
           double y_r= y_start;
           double z_r= z_start;
           double x_g= goal_wp.x;
           double y_g= goal_wp.y;
           double z_g= goal_wp.alt;

           double n_x= -1.*(x_g-x_r)*(z_g-z_r);
           double n_y= -1.*(y_g-y_r)*(z_g-z_r);
           double n_z= pow(x_g-x_r,2)+pow(y_g-y_r,2);
           assert(n_z > 0 );

           z_a= z0+ 1./n_z*( n_x*d_x+n_y*d_y );
       }
   }

}//namespace ends
