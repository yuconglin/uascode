#include "SamplerRect.hpp"
#include <boost/random.hpp>
#include <ctime>

namespace UserTypes{

   SamplerRect::~SamplerRect(){};

   void SamplerRect::SetParams(UserStructs::PlaneStateSim& st, 
	             UserStructs::MissionSimPt& goal_wp,
		     double _sig_ga)
   {
      sigma_ga= _sig_ga;
      
      double x_root= st.x;
      double y_root= st.y;
      double z_root= st.z;
      double x_goal= goal_wp.x;
      double y_goal= goal_wp.y;
      double z_goal= goal_wp.alt; 

      double Dx= x_goal-x_root;
      double Dy= y_goal-y_root;
      double Dz= z_goal-z_root;
      double r0= sqrt(Dx*Dx+Dy*Dy);
      ga0= 0.;

      x0= x_root;
      y0= y_root;
      z0= z_root;
      theta= atan2(Dy,Dx);
      x_len= r0;
      y_len= r0;
            
   }//SetParams ends

   void SamplerRect::GetSample(double& x_a,double& y_a,double& z_a,
			UserStructs::PlaneStateSim& st,
			UserStructs::MissionSimPt& goal_wp)
   {
      boost::mt19937 generator;
      static unsigned int seed = 0;
      generator.seed(static_cast<unsigned int>(std::time(0))+(++seed));
      //sample x,y
      //x
      boost::uniform_real<> x_uniform(0, x_len);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > x_uni(generator, x_uniform);
      double xc= x_uni();
      //y
      boost::uniform_real<> y_uniform(-y_len/2, y_len/2);
      boost::variate_generator<boost::mt19937&,boost::uniform_real<> > y_uni(generator, y_uniform);
      double yc= y_uni();
      //transform
      x_a= x0+ xc*cos(theta)- yc*sin(theta);
      y_a= y0+ xc*sin(theta)+ yc*cos(theta);
      double x_r= st.x; 
      double y_r= st.y;
      double z_r= st.z;
      double x_g= goal_wp.x;  
      double y_g= goal_wp.y;
      double z_g= goal_wp.alt;
      double r=sqrt(pow(x_r-x_g,2)+pow(y_r-y_g,2)+pow(z_r-z_g,2));

      if(sample_method== 0)
      {
        boost::normal_distribution<> ga_distribution(ga0, sigma_ga);
	boost::variate_generator<boost::mt19937&,boost::normal_distribution<> > ga_nor(generator, ga_distribution); 
	double ga= ga_nor();
        z_a=z0+ r*sin(ga);
      }
      else if(sample_method==1)
      {
	double d_x= x0- x_a;
	double d_y= y0- y_a;
	//get normal vector
		      
	double n_x= -1.*(x_g-x_r)*(z_g-z_r);
	double n_y= -1.*(y_g-y_r)*(z_g-z_r);
	double n_z= pow(x_g-x_r,2)+pow(y_g-y_r,2);
	assert(n_z > 0 );

	z_a= z0+ 1./n_z*( n_x*d_x+n_y*d_y );
      }
 
   }//GetSample ends


}//namespace ends

