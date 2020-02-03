
package pool.tests;

import java.awt.Graphics;  
import java.awt.image.BufferedImage;
import java.io.File;
import java.util.ArrayList;  
import java.util.List;  

import javax.swing.JFrame;  
import javax.swing.JPanel;  

import org.opencv.core.Core;  
import org.opencv.core.Mat;   
import org.opencv.core.Point;  
import org.opencv.core.Scalar;  
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;  
import org.opencv.imgproc.Imgproc;  
import org.opencv.core.CvType;  

import pool.utils.CVLoader;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;

import acm.graphics.*;
import acm.program.*;

import java.awt.event.KeyEvent;
import java.util.Arrays;

import javax.imageio.ImageIO;
import javax.swing.JButton;

public class AR8 extends GraphicsProgram implements MouseListener , MouseMotionListener , MouseWheelListener
{
 Thread th;

 GPolygon[] rect = new GPolygon[805000];
		
 		JButton play = new JButton("Play");
 		JButton reset = new JButton("Reset");
	    JButton right = new JButton("Y-");
	    JButton left = new JButton("Y+");
	    
	    JButton up = new JButton("Z+");
	    JButton down = new JButton("Z-");
	    
	    JButton front = new JButton("X+");
	    JButton back = new JButton("X-");
	    
	    JButton move_Y_up = new JButton("move_Y+");
	    JButton move_Y_down = new JButton("move_Y-");
	    
	    JButton move_Z_up = new JButton("move_Z+");
	    JButton move_Z_down = new JButton("move_Z-");
	    
	    JButton move_X_up = new JButton("move_X+");
	    JButton move_X_down = new JButton("move_X-");
	    
	    JButton arrowU = new JButton("^");
	    JButton arrowD = new JButton("v");
	    JButton arrowR = new JButton(">");
	    JButton arrowL = new JButton("<");
	    
	    JButton in = new JButton("in");
	    JButton out = new JButton("out");
	    
	    JButton hUp = new JButton("H_UP");
	    JButton hDown = new JButton("H_DOWN");
	    JButton hRight = new JButton("H_RIGHT");
	    JButton hLeft = new JButton("H_LEFT");
	    JButton hIn = new JButton("H_IN");
	    JButton hOut = new JButton("H_OUT");
	    
	    double hMove = 5;
	    
	    JButton hXCW = new JButton("H_X_CW");
	    JButton hXACW = new JButton("H_X_ACW");
	    JButton hYCW = new JButton("H_Y_CW");
	    JButton hYACW = new JButton("H_Y_ACW");
	    JButton hZCW = new JButton("H_Z_CW");
	    JButton hZACW = new JButton("H_Z_ACW");
	    
	    JButton rSX = new JButton("rSX");
	    JButton rSY = new JButton("rSY");
	    JButton rSZ = new JButton("rSZ");
	
	    JButton expS = new JButton("expS");
	    JButton contrS = new JButton("contrS");
	    
	    double hRotate = 5;	//	in Degree
	    
	        
	    static double[] xn3={0,0,0};
	    static double[] yn3={0,0,0};
	    static double[] zn3={0,0,0};
	    
	    static double[] xx3={200,0,0};
	    static double[] yx3={0,200,0};
	    static double[] zx3={0,0,200};
	    
	    static double xnx,xny ,xxx,xxy,ynx,yny,yxx,yxy,znx,zny,zxx,zxy;
	    
	    Color myColor=new Color(0,216,216);//(64,224,208);
	    
	    static double[] loc=new double[805000];
	    static double[][][] buff=new double[805000][3][2];
	    
	    ///////////////////////////////////////////////////////////////////////////////////////////////////
	  
	    static double[][] p= new double[805000][3];
	    
	    static double[] a = { -100.1, 1, 1};
	    
	    static double[] d = { 1.001, 0.0001, 0.0001 };
	 
	    
	    double dchange=0.1;
		
		double schange=20;	//5
		
		static double zoom=500;
	    
	    ///////////////////////////////////////////////////////////////////////////////////////////////////
	    
	    static double[][] v=new double[805000][2];
	    
	    static double tx,ty;
	    
	    //new_ static double radii;
	    
	    //new_ static double[] angle = new double[805000];

	    static int control;
	    
	    static int j;
	    
	    static File input = new File("plain.jpg");
	    
	    static BufferedImage bufferImg;// = ImageIO.read(input);
	    
	    GImage blank= new GImage("plain.jpg");
	    
	    GPoint gp=new GPoint(0,0);
	    
	    static double focus=90;
	    
	    int mouseflag=0;	// init state
	    int mousecounter=0;
	    int mouse_counter_controller=5;
	    
	    int mousefX, mousefY, mousetX, mousetY;
	    
	    int move=5;
	    
	    static int pointCount=0;
	    static int triCount=0;
	    
	    static double[][] COM = new double[805000][3];
	    static double[][] comDist = new double[805000][2];
	    
	    int arrowMagni;
	    
	    int lockLift=0;
	    
	    boolean playFlag = false;
	    boolean expSFlag = false;
	    boolean contrSFlag = false;
	    boolean rxSFlag = false;
	    //////////////////////////////////////////////////
	    
	    Triangle3D xAxis = new Triangle3D(new double[][] {  
			{ 0,0,0 },
			{ 0,0,0 },
			{ 100,0,0 } 	});
	    
	    Triangle3D yAxis = new Triangle3D(new double[][] {  
			{ 0,0,0 },
			{ 0,0,0 },
			{ 0,100,0 } 	});
	    
	    Triangle3D zAxis = new Triangle3D(new double[][] {  
			{ 0,0,0 },
			{ 0,0,0 },
			{ 0,0,100 } 	});
	    
	    Plane3D XAxis = new Plane3D(new double[][] {  
	    	{ 1,0,0 },
			{ 1,0,0 },
			{ 100,0,0 },
			{ 100,0,0 }		});
	    
	    Plane3D YAxis = new Plane3D(new double[][] {  
			{ 0,1,0 },
			{ 0,1,0 },
			{ 0,100,0 },
			{ 0,100,0 }		});
	    
	    Plane3D ZAxis = new Plane3D(new double[][] {  
	    	{ 0,0,1 },
			{ 0,0,1 },
			{ 0,0,100 },
			{ 0,0,100 }		});
	    
	    Plane3D pyrbase2 = new Plane3D(new double[][] {  
			{ 200,0+420,0 },
			{ 300,0+420,0 },
			{ 300,100+420,0 },
			{ 200,100+420,0}		});
	    
	    Triangle3D tri = new Triangle3D(new double[][] {  
			{ 0,20,0 },
			{ 0,0,0 },
			{ 0,0,20 },		});
	    
	    Triangle3D pyrt22 = new Triangle3D(new double[][] {  
			{ 200,0+420,0 },
			{ 200,100+420,0 },
			{ 250,50+420,500 },		});
	    
	    Triangle3D pyrt32 = new Triangle3D(new double[][] {  
			{ 200,100+420,0 },
			{ 300,100+420,0 },
			{ 250,50+420,500 },		});
	    
	    Triangle3D pyrt42 = new Triangle3D(new double[][] {  
			{ 300,0+420,0 },
			{ 300,100+420,0 },
			{ 250,50+420,500 },		});
	    
	    Plane3D pyrbase = new Plane3D(new double[][] {  
			{ 200,0,0 },
			{ 300,0,0 },
			{ 300,100,0 },
			{ 200,100,0}		});
	    
	    Triangle3D pyrt1 = new Triangle3D(new double[][] {  
			{ 200,0,0 },
			{ 300,0,0 },
			{ 250,50,500 },		});
	    
	    Triangle3D pyrt2 = new Triangle3D(new double[][] {  
			{ 200,0,0 },
			{ 200,100,0 },
			{ 250,50,500 },		});
	    
	    Triangle3D pyrt3 = new Triangle3D(new double[][] {  
			{ 200,100,0 },
			{ 300,100,0 },
			{ 250,50,500 },		});
	    
	    Triangle3D pyrt4 = new Triangle3D(new double[][] {  
			{ 300,0,0 },
			{ 300,100,0 },
			{ 250,50,500 },		});
	    ///////
	    Plane3D p1 = new Plane3D(new double[][] {  
			{ -1000,200,0 },
			{ 100000,200,0 },
			{ 100000,220,0 },
			{ -1000,220,0}		});
	    
	    Plane3D p2 = new Plane3D(new double[][] {  
			{ -1000,200+100,0 },
			{ 100000,200+100,0 },
			{ 100000,220+100,0 },
			{ -1000,220+100,0}		});
	    
	    ////////////////////////////////////////////////////////////////HELI
	    
	    double ratio = 3;
	    
	    Box3D bodyD;
	    Box3D body = new Box3D(new double[][] {  
			{ (0-4)*ratio,0,(0-2)*ratio },	
			{ (8-4)*ratio,0,(0-2)*ratio },
			{ (8-4)*ratio,0*ratio,(4-2)*ratio },
			{ (0-4)*ratio,0*ratio,(4-2)*ratio },
			
			{ (0-4)*ratio,8*ratio,(0-2)*ratio},	
			{ (8-4)*ratio,8*ratio,(0-2)*ratio },						
			{ (8-4)*ratio,8*ratio,(4-2)*ratio },
			{ (0-4)*ratio,8*ratio,(4-2)*ratio}	});
	    
	    Triangle3D[] headTriangleD;
	    Triangle3D[] headTriangle = {
	    		new Triangle3D(new double[][] {  
	    			{ (0-4)*ratio,8*ratio,(0-2)*ratio },
	    			{ (0-4)*ratio,8*ratio,(4-2)*ratio },
	    			{ (4-4)*ratio,15*ratio,(2-2)*ratio },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (8-4)*ratio,8*ratio,(0-2)*ratio},
	    			{ (8-4)*ratio,8*ratio,(4-2)*ratio },
	    			{ (4-4)*ratio,15*ratio,(2-2)*ratio },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (0-4)*ratio,8*ratio,(0-2)*ratio },
	    			{ (8-4)*ratio,8*ratio,(0-2)*ratio },
	    			{ (4-4)*ratio,15*ratio,(2-2)*ratio },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (0-4)*ratio,8*ratio,(4-2)*ratio },
	    			{ (8-4)*ratio,8*ratio,(4-2)*ratio },
	    			{ (4-4)*ratio,15*ratio,(2-2)*ratio },		})
	    };
	    
	    double lshift = 0;
	    
	    Triangle3D[] bladeLiftD;
	    Triangle3D[] bladeLift = {
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*ratio,4*ratio,(6-2)*ratio+lshift },
	    			{ (5-4)*ratio,15*ratio,(6-2)*ratio+lshift },
	    			{ (3-4)*ratio,15*ratio,(6-2)*ratio+lshift },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*ratio,4*ratio,(6-2)*ratio+lshift },
	    			{ (5-4)*ratio,-7*ratio,(6-2)*ratio+lshift },
	    			{ (3-4)*ratio,-7*ratio,(6-2)*ratio+lshift },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*ratio,4*ratio,(6-2)*ratio+lshift },
	    			{ (15-4)*ratio,5*ratio,(6-2)*ratio+lshift },
	    			{ (15-4)*ratio,3*ratio,(6-2)*ratio+lshift },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*ratio,4*ratio,(6-2)*ratio+lshift },
	    			{ (-7-4)*ratio,5*ratio,(6-2)*ratio+lshift },
	    			{ (-7-4)*ratio,3*ratio,(6-2)*ratio+lshift },		})
	    };
	 
	    Box3D tailD;
	    Box3D tail = new Box3D(new double[][] {  
	    	{ (4-0.5-4)*ratio,0,(2-0.5-2)*ratio },
	    	{ (4+0.5-4)*ratio,0,(2-0.5-2)*ratio },
	    	{ (4+0.5-4)*ratio,0,(2+0.5-2)*ratio },
	    	{ (4-0.5-4)*ratio,0,(2+0.5-2)*ratio },
			
	    	{ (4-0.2-4)*ratio,-15*ratio,(2-0.2-2)*ratio },
	    	{ (4+0.2-4)*ratio,-15*ratio,(2-0.2-2)*ratio },
	    	{ (4+0.2-4)*ratio,-15*ratio,(2+0.2-2)*ratio },
	    	{ (4-0.2-4)*ratio,-15*ratio,(2+0.2-2)*ratio },	});
	    
	    double tratio = 10;
	    double tshift = 100;
	    Triangle3D[] bladeRotateD;
	    Triangle3D[] bladeRotate = {
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*tratio,-15*tratio+tshift,(2-2)*tratio },//down
	    			{ (4-4)*tratio,-15.1*tratio+tshift,(1.5-2)*tratio },
	    			{ (4-4)*tratio,-14.9*tratio+tshift,(1.5-2)*tratio },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*tratio,-15*tratio+tshift,(2-2)*tratio },//up
	    			{ (4-4)*tratio,-15.1*tratio+tshift,(2.5-2)*tratio },
	    			{ (4-4)*tratio,-14.9*tratio+tshift,(2.5-2)*tratio },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*tratio,-15*tratio+tshift,(2-2)*tratio },//right
	    			{ (4-4)*tratio,-15.5*tratio+tshift,(2.1-2)*tratio },
	    			{ (4-4)*tratio,-15.5*tratio+tshift,(1.9-2)*tratio },		}),
	    		new Triangle3D(new double[][] {  
	    			{ (4-4)*tratio,-15*tratio+tshift,(2-2)*tratio },//left
	    			{ (4-4)*tratio,-14.5*tratio+tshift,(2.1-2)*tratio },
	    			{ (4-4)*tratio,-14.5*tratio+tshift,(1.9-2)*tratio },		})
	    };
	    
	    Helicopter3D h1;// = new Helicopter3D(body, headTriangle, bladeLift, tail, bladeRotate);
	    
	    //////////////////////////////////////////////////////////////////////////////////HELI
	    
	    Sphere sp1= new Sphere( 4, new double[] {50,0,0}, 10, 10);
	    Sphere[] sp = new Sphere[5000];
	    
	    static int succ_tri = 0;    
	    


        Plane3D pitch = new Plane3D(new double[][] {
            new double[] { -200,0,-15 },
            new double[] { -200,2000,-15 },
            new double[] { 200,2000,-15 },
            new double[] { 200,0,-15 }    });
        
	    public static double dirTheta = 0.1, dirPhi =90.1, dirRadii = 5;
	    public static double dTheta = 5, dPhi = 5, dRadii = 5;
	    
	    public static boolean removeFlag = false, button = true;
	    
	    ///////////////////////////////////////////////////////////
	    
	    static Mat _webcam_image;     
	    static Mat webcam_image;  
	    
	    /////////////////////////////////////////////////////////
	    
	    double[] b1Position = new double[3];
	    double[] b2Position = new double[3];
	    double[] b12Center = new double[3];
	    
	    double ballTheta, ballPhi;
	    
	    /////////////////////////////////////////////////////////
public void init()
{ 
	
	th=new Thread(this);
	
	d[0] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.cos(Math.toRadians(dirTheta));	// sinphi costheta
	d[1] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.sin(Math.toRadians(dirTheta));	// sinphi sintheta
	d[2] = dirRadii * Math.cos(Math.toRadians(dirPhi));	// cosphi
	
	a[0] = -d[0];
	a[1] = -d[1];
	a[2] = -d[2];	
	
	add(play, EAST);
	
	add(reset, EAST);
	
	add(in,EAST);
	add(out,EAST);
	
	add(arrowU,EAST);
	add(arrowD,EAST);
	add(arrowR,EAST);
	add(arrowL,EAST);
	
	add(front, EAST);
    add(back, EAST);
	add(left, EAST);
    add(right, EAST);    
    add(up, EAST);
    add(down, EAST);    
    
    add(move_X_up,EAST);
    add(move_X_down,EAST);
    add(move_Y_up,EAST);
    add(move_Y_down,EAST);
    add(move_Z_up,EAST);
    add(move_Z_down,EAST);
    
    add(hUp,WEST);
    add(hDown,WEST);
    add(hRight,WEST);
    add(hLeft,WEST);
    add(hIn,WEST);
    add(hOut,WEST);
    
    

    
    addActionListeners();     
    addMouseListener( this );
    addMouseMotionListener( this );
    addMouseWheelListener(this);
    			
	setVisible(true);
    
}

public static double dot( double[] a,double[] b )
{	
	return ( a[0]*b[0]+a[1]*b[1]+a[2]*b[2] );
}
	
public static double[] cross(double[] a,double[] b)
{
	double[] crs = {0,0,0};
	
	crs[0]=a[1]*b[2] - b[1]*a[2] ;
	crs[1]=b[0]*a[2] - a[0]*b[2] ;
	crs[2]=a[0]*b[1] - b[0]*a[1] ;
	
	return crs;	
}

public static double mod(double[] v)
{
	double ln=(double) Math.sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
	
	return ln;
}

public static double[] diff(double[] a,double[] b)
{
	double[] d={ 0,0,0, };
	
	d[0]=b[0] - a[0];
	d[1]=b[1] - a[1];
	d[2]=b[2] - a[2];
	
	return d;
}

public static double per_dist(double[] p,double[] a, double[] d)
{
	double width = 0;
	
	width=( mod( cross( diff( p , a ), d ) ) )/( mod(d) );
	
	return width;
}

public static double per_dist_plane(double[] p,double[] a,double[] d)//w
{
	double D;
	
	D=a[0]*d[0]+a[1]*d[1]+a[2]* d[2];
	
	return ( (Math.abs( dot(p,d) - D ))/( mod(d) ) );	
}

public static double depth(double[] p,double[] a, double[] d )
{
	return (Math.abs( dot( diff( p,a ) , d ) / ( mod(d) ) ));
}

public static double displ( double[] p,double[] a, double[] d )
{	
	double dtemp=depth(p,a,d);
	
	if(dtemp==0)
		dtemp=0.0000001;	//ANY SMALL
	
	return ( per_dist(p,a,d) )/( dtemp );
}

public static double[] unit( double[] v)
{
	double md=mod(v);
	double[] unt= { 0,0,0 };
	
	unt[0]=v[0]/md;
	unt[1]=v[1]/md;
	unt[2]=v[2]/md;
	
	return unt;
}

public static double get_phi(double[] v)
{
	double phi=0;
	
	phi=Math.toDegrees( Math.acos( v[2]/(Math.sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] ))) );
	
	return phi;
}

public static double get_theta(double[] v)
{
	double theta=0;
	
	if( v[0]==0 & v[1]==0)
		return theta;	//exception
	
	theta=Math.toDegrees( Math.acos( v[0]/(Math.sqrt( v[0]*v[0] + v[1]*v[1] ))) );
	
	if( v[1]<0 )
		theta=360-theta;
	
	return theta;
}


public static double[] get_diff_phi( double[] d, double[] p)
{
	double phi_d=get_phi(d);
	double phi_p=get_phi(p);
	
	double[] diff_phi={0,0};
	
	double[] use_diff_theta=get_diff_theta(d,p);
	
	if( phi_d<90 && phi_p<90 )
	{
		if( Math.abs(use_diff_theta[0])>90 )
		{
				diff_phi[0]= phi_d + phi_p ;
				diff_phi[1]=1;
		}
		else if( Math.abs(use_diff_theta[0])<90 )
		{
			diff_phi[0]=phi_d - phi_p;
			
			if( diff_phi[0]>0 )
				diff_phi[1]=1;
			else
				diff_phi[1]=-1;
		}
	}
	else if( phi_d>90 && phi_p>90 )
	{
		if( use_diff_theta[0]>90 )
		{
				diff_phi[0]= 360 - phi_d - phi_p ;
				diff_phi[1]=-1;
		}
		else if( Math.abs(use_diff_theta[0])<90 )
		{
			diff_phi[0]=phi_d - phi_p;
			
			if( diff_phi[0]>0 )
				diff_phi[1]=1;
			else
				diff_phi[1]=-1;
		}
	}
	else
	{		
		if( Math.abs(use_diff_theta[0])>90 )
		{			
			diff_phi[0]= phi_d + phi_p ;
			diff_phi[1]=1;//or-1
		}
		else if( Math.abs(use_diff_theta[0])<90 )
		{
			diff_phi[0]=phi_d - phi_p;
			
			if( diff_phi[0]>0 )
				diff_phi[1]=1;
			else
				diff_phi[1]=-1;
		}		
	}
	
	return diff_phi;
}


public static double[] get_diff_theta( double[]d, double[] p)
{
	double[] diff_theta={0,0};
	
	double theta_d=get_theta(d);
	double theta_p=get_theta(p);
	
	if( theta_d<90 && theta_p>270 )
	{
		diff_theta[0]=theta_d + 360 - theta_p;
		diff_theta[1]=1;
	}
	else if( theta_d>270 && theta_p<90 )
	{
		diff_theta[0]=(theta_p + 360 - theta_d)*(-1);
		diff_theta[1]=-1;
	}
	else
	{
		diff_theta[0]= theta_d - theta_p;	
		
		if( diff_theta[0]>0 )
			diff_theta[1]=1;
		else
			diff_theta[1]=-1;
	}
	
	return diff_theta;
}

public static double[] get_rel_vect(double[] d, double[] p)
{
	double[] rel_vect={0,0,0};
	
	double phi_d=get_phi(d);
	double phi_p=get_phi(p);
	
	double[] diff_phi={0,0};
	
	double[] use_diff_theta=get_diff_theta(d,p);
	
	double[] rel_unit_vect={0,0,0};
	
	double limit = 90;
	
	if( phi_d<90 && phi_p<90 )
	{
		if( Math.abs(use_diff_theta[0])>limit )
		{			
			rel_vect[0]=180 - Math.abs(use_diff_theta[0]);//ts_6_1
			
			rel_vect[0]= use_diff_theta[1]*rel_vect[0];	
			
			rel_vect[1]=phi_d + phi_p;
			
			rel_unit_vect[2]=-3;
		}
		else if( Math.abs(use_diff_theta[0])<90 )
		{
			diff_phi[0]=phi_d - phi_p;
			
			if( diff_phi[0]>0 )
				diff_phi[1]=1;
			else
				diff_phi[1]=-1;
			
			rel_vect[1]=diff_phi[0];
			
			rel_vect[0]= use_diff_theta[0];
		}
	}
	else if( phi_d>90 && phi_p>90 )
	{
		if( Math.abs(use_diff_theta[0])>limit )
		{
			rel_vect[1]= (360 - phi_d - phi_p)*(-1);
			
			rel_vect[0]=180 - use_diff_theta[0];	
			
			rel_vect[0]= use_diff_theta[1]*rel_vect[0];	
			
			rel_unit_vect[2]=-3;
		}
		else if( Math.abs(use_diff_theta[0])<90 )
		{
			diff_phi[0]=phi_d - phi_p;
			
			if( diff_phi[0]>0 )
				diff_phi[1]=1;
			else
				diff_phi[1]=-1;
			
			rel_vect[1]=diff_phi[0];
			
			rel_vect[0]= use_diff_theta[0];
		}
	}
	else
	{
		if( Math.abs(use_diff_theta[0])>limit )
		{
			
				diff_phi[0]= phi_d + phi_p ;
				diff_phi[1]=1;
				
				rel_unit_vect[2]=-3;
		}
		else if( Math.abs(use_diff_theta[0])<90 )
		{
			diff_phi[0]=phi_d - phi_p;
			
			if( diff_phi[0]>0 )
				diff_phi[1]=1;
			else
				diff_phi[1]=-1;
		}
		
		rel_vect[1]=diff_phi[0];
		
		rel_vect[0]= use_diff_theta[0];
	}
	
	double rel_vect_magn=Math.sqrt(rel_vect[0]*rel_vect[0] + rel_vect[1]*rel_vect[1]);
	
	rel_unit_vect[0]=rel_vect[0]/rel_vect_magn;
	rel_unit_vect[1]=rel_vect[1]/rel_vect_magn;
	
	 return rel_unit_vect;	
}

///////////////////////////////////////////////////////////////////////////////////////////
public static void trans()
{
	int new_count = -1;
	double radii;
	
	succ_tri = 0;
	
	for(int i=0; i < pointCount; i += 3, succ_tri++)
	{		
		radii = displ( p[i], a, d );
		
		double[] rel_v=get_rel_vect(d, diff(a,p[i]) );				
		
		tx= zoom*radii*rel_v[0];
		ty= zoom*radii*rel_v[1];
		
		if(rel_v[2] == 0)
		{				
			v[++new_count][0] = 640 + tx;
			v[new_count][1] = 480 - ty;
			
			//System.out.println("x = "+v[new_count][0]+", y = "+v[new_count][1]);
			
			radii = displ( p[i+1], a, d );			
			rel_v=get_rel_vect(d, diff(a,p[i+1]) );				
			
			tx= zoom*radii*rel_v[0];
			ty= zoom*radii*rel_v[1];
			
			v[++new_count][0] = 640 + tx;
			v[new_count][1] = 480 - ty;
			
			//System.out.println("x = "+v[new_count][0]+", y = "+v[new_count][1]);
			
			radii = displ( p[i+2], a, d );			
			rel_v=get_rel_vect(d, diff(a,p[i+2]) );				
			
			tx= zoom*radii*rel_v[0];
			ty= zoom*radii*rel_v[1];	
			
			v[++new_count][0] = 640 + tx;
			v[new_count][1] = 480 - ty;
			//
			//System.out.println("x = "+v[new_count][0]+", y = "+v[new_count][1]);
		}		
		else
		{			
			v[++new_count][0] =0;
			v[new_count][1] =0;
									
			v[++new_count][0] =0;
			v[new_count][1] =0;
			
			v[++new_count][0] =0;
			v[new_count][1] =0;
			
		}
	}
}

static void addTriangle(double[][] trig)
{
	p[pointCount]=new double[3];
	p[pointCount +1]=new double[3];
	p[pointCount +2]=new double[3];
	
	p[pointCount]=trig[0];
	p[pointCount+1]=trig[1];
	p[pointCount+2]=trig[2];
	
	pointCount+=3;
	triCount+=1;
}

static void addPlane(double[][] plane)
{
	double[][] triangle = new double[3][3];
	
	triangle[0] = plane[1];
	triangle[1] = plane[0];
	triangle[2] = plane[3];
	
	addTriangle(triangle);
	
	triangle[0] = plane[1];
	triangle[1] = plane[2];
	triangle[2] = plane[3];
	
	addTriangle(triangle);
	
}

static void addBox(double[][] box)
{
	double[][] plane = new double[6][3];
	
	//base
	plane[0] = box[0];
	plane[1] = box[1];
	plane[2] = box[2];
	plane[3] = box[3];
	
	addPlane(plane);
	
	//top
	plane[0] = box[4];
	plane[1] = box[5];
	plane[2] = box[6];
	plane[3] = box[7];
	
	addPlane(plane);
	
	//YZ near	
	plane[0] = box[3];
	plane[1] = box[0];
	plane[2] = box[4];
	plane[3] = box[7];
	
	addPlane(plane);
	
	//YZ far
	plane[0] = box[2];
	plane[1] = box[1];
	plane[2] = box[5];
	plane[3] = box[6];
	
	addPlane(plane);
	
	// XZ near
	plane[0] = box[0];
	plane[1] = box[1];
	plane[2] = box[5];
	plane[3] = box[4];
	
	addPlane(plane);
	
	//XZ far
	plane[0] = box[3];
	plane[1] = box[2];
	plane[2] = box[6];
	plane[3] = box[7];
	
	addPlane(plane);
}
///////////////////////////////////////////   EDIT

static void editTriangle(double[][] trig, int editLoc)
{
	p[editLoc]=trig[0];
	p[editLoc+1]=trig[1];
	p[editLoc+2]=trig[2];
	
}

static void editPlane(double[][] plane, int editLoc)
{
	double[][] triangle = new double[3][3];
	
	triangle[0] = plane[1];
	triangle[1] = plane[0];
	triangle[2] = plane[3];
	
	editTriangle(triangle, editLoc);
	
	triangle[0] = plane[1];
	triangle[1] = plane[2];
	triangle[2] = plane[3];
	
	editTriangle(triangle, editLoc + 3);
	
}

static void editBox(double[][] box, int editLoc)
{
	double[][] plane = new double[6][3];
	
	//base
	plane[0] = box[0];
	plane[1] = box[1];
	plane[2] = box[2];
	plane[3] = box[3];
	
	editPlane(plane, editLoc);
	
	//top
	plane[0] = box[4];
	plane[1] = box[5];
	plane[2] = box[6];
	plane[3] = box[7];
	
	editPlane(plane, editLoc + 6);
	
	//YZ near	
	plane[0] = box[3];
	plane[1] = box[0];
	plane[2] = box[4];
	plane[3] = box[7];
	
	editPlane(plane, editLoc + 12);
	
	//YZ far
	plane[0] = box[2];
	plane[1] = box[1];
	plane[2] = box[5];
	plane[3] = box[6];
	
	editPlane(plane, editLoc + 18);
	
	// XZ near
	plane[0] = box[0];
	plane[1] = box[1];
	plane[2] = box[5];
	plane[3] = box[4];
	
	editPlane(plane, editLoc + 24);
	
	//XZ far
	plane[0] = box[3];
	plane[1] = box[2];
	plane[2] = box[6];
	plane[3] = box[7];
	
	editPlane(plane, editLoc + 36);
}

///////////////////////////////////////////   EDIT END

public void paint(Graphics g)
{		
	blank.addMouseListener(this);
	blank.addMouseMotionListener(this);
	
	Imgproc.cvtColor(webcam_image, webcam_image, Imgproc.COLOR_RGB2BGR);
	//add(blank,0,0);
	
	byte[] data1 = new byte[webcam_image.rows() * webcam_image.cols() * (int)(webcam_image.elemSize())];
	webcam_image.get(0, 0, data1);
   
	bufferImg = new BufferedImage(webcam_image.cols(),webcam_image.rows(), BufferedImage.TYPE_3BYTE_BGR);
	
	
	
	bufferImg.getRaster().setDataElements(0, 0, webcam_image.cols(), webcam_image.rows(), data1);
	
	blank = new GImage(bufferImg);
	add(blank,0,0);
	
	
    int m;
    for(int t=0;t < succ_tri; t++)
    {	
    	rect[t]=new GPolygon();    	
    	m=t*3;
    	   	
    	rect[t].addVertex(v[m][0], v[m][1]);
    	rect[t].addVertex(v[m+1][0], v[m+1][1]);		
    	rect[t].addVertex(v[m+2][0], v[m+2][1]);  	
    }
    
    
    preCalc();
    
    for( int z=0;z<succ_tri;z++)
    {   	    	
    	int x = (int) comDist[z][1];
    	
    	rect[x].setFillColor(myColor);
		rect[x].setFilled(true); 
		rect[x].setVisible(true);
		add(rect[x]);
    }	
}

static void sortDist()
{
	int c,d;
	double swap;
	
	for (c = 0; c < ( succ_tri - 1 ); c++) 
	{
	      for (d = 0; d < succ_tri - c - 1; d++) 
	      {
	        if (comDist[d][0] < comDist[d+1][0]) 
	        {
	          swap       = comDist[d][0];
	          comDist[d][0]   = comDist[d+1][0];
	          comDist[d+1][0] = swap;
	          
	          swap       = comDist[d][1];
	          comDist[d][1]   = comDist[d+1][1];
	          comDist[d+1][1] = swap;	          
	        }
	      }
	 }
}
	
static void calcDist()
{
	double[] comass = new double[3];
	
	for(int i=0;i<succ_tri;i++)
	{
		comass[0]=COM[i][0];
		comass[1]=COM[i][1];
		comass[2]=COM[i][2];
						
		comDist[i][0] = mod( diff( a, comass ) );
		comDist[i][1] = i;		
	}
}

static void calcCOM()
{
	for(int i=0;i<succ_tri;i++)
	{
		COM[i][0] = (p[i*3][0] + p[i*3 + 1][0] + p[i*3 + 2][0])/3;
		COM[i][1] = (p[i*3][1] + p[i*3 + 1][1] + p[i*3 + 2][1])/3;
		COM[i][2] = (p[i*3][2] + p[i*3 + 1][2] + p[i*3 + 2][2])/3;
	}
}

static void preCalc()
{
	calcCOM();
	calcDist();
	sortDist();
}

public void keyReleased(KeyEvent arg0) {}

public void keyTyped(KeyEvent ke) {
char ch=ke.getKeyChar();
}

public void actionPerformed(ActionEvent e) {
	
    if(e.getSource() == front ) 
    { 
      if( (d[0] + dchange )==0 )
    	  d[0]=dchange;
      else
    	  d[0]+=dchange;
    }
    else if(e.getSource() == back ) 
    {
    	if( (d[0] - dchange )==0 )
    		d[0]=0-dchange;
    	else
    		d[0]-=dchange;
    }
    else if(e.getSource() == up ) 
    {
    	if( (d[2] + dchange )==0 )
    		d[2]=dchange;
    	else
    		d[2]+=dchange;
    }
    else if(e.getSource() == down ) 
    {
    	if( (d[2] - dchange )==0 )
    		d[2]=0-dchange;
    	else
    		d[2]-=dchange;
    }
    else if(e.getSource() ==  left ) 
    {
    	if( (d[1] + dchange )==0 )
    		d[1]=dchange;
    	else
    		d[1]+=dchange;
    }
    else if(e.getSource() == right ) 
    {
    	if( (d[1] - dchange )==0 )
    		d[1]=0-dchange;
    	else
    		d[1]-=dchange;
    }
    else if(e.getSource() == move_X_up ) 
    {
    		a[0]+=schange;
    }
    else if(e.getSource() == move_X_down ) 
    {
    		a[0]-=schange;
    }
    else if(e.getSource() == move_Y_up  ) 
    {
    		a[1]+=schange;
    }
    else if(e.getSource() ==  move_Y_down ) 
    {
    		a[1]-=schange;
    }
    else if(e.getSource() == move_Z_up  ) 
    {
    		a[2]+=schange;
    }
    else if(e.getSource() == move_Z_down  ) 
    {
    		a[2]-=schange;
    }
    else if(e.getSource() == arrowU)
    {
    	direct( 0, 0, 0, arrowMagni );
    }
    else if(e.getSource() == arrowD)
    {
    	direct( 0, 0, 0, -arrowMagni );
    }
    else if(e.getSource() == arrowR)
    {
    	direct( 0, 0, arrowMagni, 0 );
    }
    else if(e.getSource() == arrowL)
    {
    	direct( 0, 0, -arrowMagni, 0 );
    }
    else if( e.getSource() == in)
    {
    	a[0] = a[0] + move;
	    a[1] = a[1] + move;
	    a[2] = a[2] + move;
    }
    else if( e.getSource() == out)
    {    	
    	a[0] = a[0] - move;
	    a[1] = a[1] - move;
	    a[2] = a[2] - move;
    }/////////////////////////////////////////////////
    else if( e.getSource() == hUp )
    {
    	if(dirPhi < 180)
    		dirPhi += dPhi;
    	
    	d[2] = dirRadii * Math.cos(Math.toRadians(dirPhi));
    	a[2] = -d[2];
    }
    else if( e.getSource() == hDown )
    {
    	if(dirPhi > 0)
    		dirPhi -= dPhi;
    	
    	d[2] = dirRadii * Math.cos(Math.toRadians(dirPhi));
    	a[2] = -d[2];
    }
    else if( e.getSource() == hRight )
    {
    	if(dirTheta + dTheta < 360)
    		dirTheta += dTheta;
    	else
    		dirTheta = 0;
    	
    	d[0] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.cos(Math.toRadians(dirTheta));	// sinphi costheta
    	d[1] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.sin(Math.toRadians(dirTheta));	// sinphi sintheta
    	
    	a[0] = -d[0];
    	a[1] = -d[1];
    }
    else if( e.getSource() == hLeft )
    {
    	if(dirTheta - dTheta > 0)
    		dirTheta -= dTheta;
    	else
    		dirTheta = 360;
    	
    	d[0] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.cos(Math.toRadians(dirTheta));	// sinphi costheta
    	d[1] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.sin(Math.toRadians(dirTheta));	// sinphi sintheta
    	
    	a[0] = -d[0];
    	a[1] = -d[1];
    }
    else if( e.getSource() == hIn )
    {
    	dirRadii -= dRadii;
    	
    	d[0] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.cos(Math.toRadians(dirTheta));	// sinphi costheta
    	d[1] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.sin(Math.toRadians(dirTheta));	// sinphi sintheta
    	d[2] = dirRadii * Math.cos(Math.toRadians(dirPhi));	// cosphi
    	
    	a[0] = -d[0];
    	a[1] = -d[1];
    	a[2] = -d[2];
    }
    else if( e.getSource() == hOut )
    {
    	dirRadii += dRadii;
    	
    	d[0] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.cos(Math.toRadians(dirTheta));	// sinphi costheta
    	d[1] = dirRadii * Math.sin(Math.toRadians(dirPhi))*Math.sin(Math.toRadians(dirTheta));	// sinphi sintheta
    	d[2] = dirRadii * Math.cos(Math.toRadians(dirPhi));	// cosphi
    	
    	a[0] = -d[0];
    	a[1] = -d[1];
    	a[2] = -d[2];
    }
    else if( e.getSource() == play )
    {
    	button = false;
    	
    	if(playFlag == false)
    		playFlag = true;
    	else
    		playFlag = false;
    }
    else if( e.getSource() == reset )
    {
    }
    
    trans();
	
	for(j=0;j<pointCount;j++)
		loc[j]=0;
	
    repaint();
}


	static void direct(int mousefX,int mousefY,int mousetX,int mousetY)
	{
		int dTheta = mousetX - mousefX;
		int dPhi = mousetY - mousefY;
		
		double theta = get_theta(d);
		double phi = get_phi(d)*(-1);	
		
		double nowTheta = 1;
		double nowPhi = 85;
		
		if( theta + dTheta > 360 )
		{
			nowTheta = 1;
		}
		else if( theta + dTheta < 0 )
		{
			nowTheta = 359;
		}
		else
		{
			nowTheta = theta + dTheta;
		}
		
		
		if( phi + dPhi > 180 )
		{
			nowTheta = theta;
		}
		else if( phi + dPhi < 0 )
		{
			nowTheta = theta;
		}
		else
		{
			nowPhi = phi + dPhi;
		}
		
		d[0] = Math.sin(Math.toRadians(nowPhi))*Math.cos(Math.toRadians(nowTheta));	// sinphi costheta
		d[1] = Math.sin(Math.toRadians(nowPhi))*Math.sin(Math.toRadians(nowTheta));	// sinphi sintheta
		d[2] = Math.cos(Math.toRadians(nowPhi));	// cosphi
		
		
	}
	
	
	public void mouseDragged(MouseEvent pointer)
	{		
		if(mousecounter < mouse_counter_controller)
		{
			
		}
		else
		{
			mousecounter=0;
			
			mousetX=pointer.getX();
			mousetY=pointer.getY();
			
			direct( mousefX, mousefY, mousetX, mousetY );
			
			trans();
			repaint();
			
			mousefX=pointer.getX();
			mousefY=pointer.getY();			
		}
		
		mousecounter++;
	}

	public void mousePressed(MouseEvent arg0)
	{		
		mousefX=arg0.getX();
		mousefY=arg0.getY();
		
		mousecounter=0;
	}

	@Override
	public void mouseWheelMoved(MouseWheelEvent e) {

		int notches = e.getWheelRotation();
	   
	    a[0] = a[0] + move*notches;
	    a[1] = a[1] + move*notches;
	    a[2] = a[2] + move*notches;
	    	    
	    trans();
	    repaint();
	}
	
public void run()
{	
		
		//addPlane(XAxis.initPlane3D(this));
		
		//addPlane(YAxis.initPlane3D(this));
		
		//addPlane(ZAxis.initPlane3D(this));
		
		//h1.initHelicopter3D(this);
		//h1.seek(new double[] {50, 0, 0});
		
		int nowCount = AR8.pointCount;
		
		//sp1.initSphere(this);
		
		removeFlag = true;
		
		CVLoader.load();
		
		_webcam_image=new Mat();     
	    webcam_image=new Mat();  
	    
	    Mat _webcam_image_org=new Mat();     
	    Mat webcam_image_org=new Mat(); 
	    
	    int[] rows = new int[2]; 
        int[] elemSize = new int[2];  
        float[][][] data2 = new float[2][2][];
        
        int[] _rows = new int[2]; 
        int[] _elemSize = new int[2];  
        float[][][] _data2 = new float[2][2][];
        
        boolean isB1 = false, isB2 = false, isB1L = false, isB1R = false, isB2L = false, isB2R = false;
		
        int yRMin = 10, yRMax = 80;
        int gRMin = 10, gRMax = 80;
        
        int detectLim = 50;
	    
		JFrame frame1 = new JFrame("Left");  
	     frame1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);  
	     frame1.setSize(1280,960);  
	     frame1.setBounds(0, 0, frame1.getWidth(), frame1.getHeight());  
	     Panel panel1 = new Panel();  
	     frame1.setContentPane(panel1);  
	     frame1.setVisible(true);  
	     
	     JFrame frame4 = new JFrame("Threshold");  
	     frame4.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);  
	     frame4.setSize(1280,960);  
	     Panel panel4 = new Panel();  
	     frame4.setContentPane(panel4);      
	     //frame4.setVisible(true);  
	     VideoCapture capture =new VideoCapture(0);  
	     
	     capture.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, 1280);
		 capture.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, 960);
		 
	     Mat hsv_image=new Mat();  
	     Mat thresholdedY;//=new Mat(960, 1280, CvType.CV_8UC3); 
	     Mat thresholdedG;//=new Mat(960, 1280, CvType.CV_8UC3);
	     Mat _thresholdedY;//=new Mat(960, 1280, CvType.CV_8UC3);
	     Mat _thresholdedG;//=new Mat(960, 1280, CvType.CV_8UC3); 
	      capture.read(webcam_image);  
	      frame1.setSize(webcam_image.width()+40,webcam_image.height()+60);  
	      frame4.setSize(webcam_image.width()+40,webcam_image.height()+60);  
	     Mat array255=new Mat(960,1280,CvType.CV_8UC1);  
	     array255.setTo(new Scalar(255));  
	     
	     Mat distance=new Mat(webcam_image.height(),webcam_image.width(),CvType.CV_8UC1);  
	     
	     List<Mat> lhsv = new ArrayList<Mat>(3);      
	     Mat circles = new Mat();
	     
	     Scalar hsv_minY = new Scalar(15, 50, 50, 0); //Orange 
	     Scalar hsv_maxY = new Scalar(19, 255, 255, 0);  
	     
	     Scalar hsv_minG = new Scalar(50, 50, 50, 0);  
	     Scalar hsv_maxG = new Scalar(70, 255, 255, 0);  
	     
	     //Scalar hsv_minY = new Scalar(20, 254, 250, 0);  
	     //Scalar hsv_maxY = new Scalar(25, 255, 255, 0); 
	     
	     byte[] data = new byte[] {(byte) 155, (byte) 161, (byte) 131};
	          ////////////////////////////////////////////////////////////     
	     JFrame _frame1 = new JFrame("Right");  
	     _frame1.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);  
	     _frame1.setSize(1280,960);  
	     _frame1.setBounds(0, 0, _frame1.getWidth(), _frame1.getHeight());  
	     Panel _panel1 = new Panel();  
	     _frame1.setContentPane(_panel1);  
	     _frame1.setVisible(true);  
	     
	     JFrame _frame4 = new JFrame("Threshold");  
	     _frame4.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);  
	     _frame4.setSize(1280,960);  
	     Panel _panel4 = new Panel();  
	     _frame4.setContentPane(_panel4);      
	     //_frame4.setVisible(true);  
	     //-- 2. Read the video stream  
	     VideoCapture _capture =new VideoCapture(2); 
	     
	     _capture.set(Highgui.CV_CAP_PROP_FRAME_WIDTH, 1280);
		 _capture.set(Highgui.CV_CAP_PROP_FRAME_HEIGHT, 960);
	     
	     
	     Mat _hsv_image=new Mat();  
	     
	     
	      _capture.read(_webcam_image);  
	      _frame1.setSize(_webcam_image.width()+40,_webcam_image.height()+60);  
	      _frame4.setSize(_webcam_image.width()+40,_webcam_image.height()+60);  
	     Mat _array255=new Mat(960,1280,CvType.CV_8UC1);  
	     _array255.setTo(new Scalar(255));  
	     
	     Mat _distance=new Mat(_webcam_image.height(),_webcam_image.width(),CvType.CV_8UC1);  
	     
	     List<Mat> _lhsv = new ArrayList<Mat>(3);      
	     Mat _circles = new Mat();
	     
	     double[] _data=new double[3]; 
	    
	    ////////////////////////////////////////////////////////////
	    
	    int detectCount = 0;
	    //if( capture.isOpened() && _capture.isOpened())  
	    
	    trans();
	    repaint();
	      
	     while( capture.grab() && _capture.grab() && button )  
	     {  
	    	 thresholdedY=new Mat(960, 1280, CvType.CV_8UC3); 
		     thresholdedG=new Mat(960, 1280, CvType.CV_8UC3);
		     _thresholdedY=new Mat(960, 1280, CvType.CV_8UC3);
		     _thresholdedG=new Mat(960, 1280, CvType.CV_8UC3);
	   	 //try {
			//Thread.sleep(200);
		//} catch (InterruptedException e) {
			// TODO Auto-generated catch block
		//	e.printStackTrace();
		//}
	   	  
	       //capture.read(webcam_image);  
	       //_capture.read(_webcam_image); 
	       
	   	capture.retrieve(webcam_image);  
	       _capture.retrieve(_webcam_image);
	       
	       for(int i = 0; i < webcam_image.rows(); i++)
				for(int j = 0; j < webcam_image.cols(); j++)
				{
					thresholdedY.put(i, j, new byte[]{(byte) 0, (byte) 0, (byte) 0});
					_thresholdedY.put(i, j, new byte[]{(byte) 0, (byte) 0, (byte) 0});
					thresholdedG.put(i, j, new byte[]{(byte) 0, (byte) 0, (byte) 0});
					_thresholdedG.put(i, j, new byte[]{(byte) 0, (byte) 0, (byte) 0});
				}
	       
	       float[] h;
	       int[] in = new int[3];
	       
	       for(int i = 0; i < webcam_image.rows(); i++)
				for(int j = 0; j < webcam_image.cols(); j++)
				{
					webcam_image.get(i, j, data);
		
					in[0] = data[0] & 0xFF;
					data[0] = (byte) in[0];
		
					in[1] = data[1] & 0xFF;
					data[1] = (byte) in[1];
	
					in[2] = data[2] & 0xFF;
					data[2] = (byte) in[2];
		
					h = HSV(in);
		
					//if(h[0] > 35 && h[0] < 65 && h[1] > 60)
					if(h[0] > 10 && h[0] < 25 && h[1] > 60 && h[2] > 40)
					{
						thresholdedY.put(i, j, new byte[]{(byte) 255, (byte) 255, (byte) 255});
					}
		
					//if(h[0] > 85 && h[0] < 175 && h[1] > 60 )
					//if(h[0] > 170 && h[0] < 195 && h[2] > 50)// && h[2] > 30)
					if(h[0] > 60 && h[0] < 80 && h[1] > 30)// && h[2] > 30)
					{
						thresholdedG.put(i, j, new byte[]{(byte) 255, (byte) 255, (byte) 255});
					}
					
					//////////////////////////////
					
					_webcam_image.get(i, j, data);
					
					in[0] = data[0] & 0xFF;
					data[0] = (byte) in[0];
		
					in[1] = data[1] & 0xFF;
					data[1] = (byte) in[1];
	
					in[2] = data[2] & 0xFF;
					data[2] = (byte) in[2];
		
					h = HSV(in);
		
					if(h[0] > 10 && h[0] < 25 && h[1] > 60 && h[2] > 40)
					{
						_thresholdedY.put(i, j, new byte[]{(byte) 255, (byte) 255, (byte) 255});
					}
		
					if(h[0] > 60 && h[0] < 80 && h[1] > 30)
					{
						_thresholdedG.put(i, j, new byte[]{(byte) 255, (byte) 255, (byte) 255});
					}
				}
	       Imgproc.cvtColor(_thresholdedY, _thresholdedY, Imgproc.COLOR_BGR2GRAY);
	       Imgproc.cvtColor(thresholdedY, thresholdedY, Imgproc.COLOR_BGR2GRAY);
	       Imgproc.cvtColor(_thresholdedG, _thresholdedG, Imgproc.COLOR_BGR2GRAY);
	       Imgproc.cvtColor(thresholdedG, thresholdedG, Imgproc.COLOR_BGR2GRAY);
	       
	       //System.out.println("channels = "+_thresholdedY.channels());
	       
	      
	       
	       //thresholdedY.convertTo(thresholdedY, CvType.CV_8UC1);
	       //_thresholdedY.convertTo(_thresholdedY, CvType.CV_8UC1); 
	       //thresholdedG.convertTo(thresholdedG, CvType.CV_8UC1); 
	       //_thresholdedG.convertTo(_thresholdedG, CvType.CV_8UC1); 
	       
	      // System.out.println("channels = "+_thresholdedY.channels());
	       
	       //if( !webcam_image.empty() && !_webcam_image.empty())  
	         
	       	
	        ///////////////////////////
	        	int t1 = 0, t2=0, t3=0, t4=0;
	        	
	        	//data2 = new float[2][2][];
	        	//_data2 = new float[2][2][];
	        	
	        	circles = new Mat();
	        	_circles = new Mat();
	        	
	           
	            //Core.inRange(_distance,new Scalar(0.0), new Scalar(150.0), _thresholdedY);  
	            //Core.bitwise_and(_thresholded, _thresholded2, _thresholded);
	        	Imgproc.blur(_thresholdedY, _thresholdedY, new Size(6, 6));  
	        	//Imgproc.Canny(_thresholdedY, _thresholdedY, 0, 20 * ratio);
	        	//Imgproc.HoughLinesP(_thresholdedY, _thresholdedY, 1, Math.PI / 180, 50, 50, 10);
	        	  
	            Imgproc.HoughCircles(_thresholdedY, _circles, Imgproc.CV_HOUGH_GRADIENT, 2, _thresholdedY.height()/4, 500, detectLim, yRMin, yRMax);   
	           
	             _rows[0] = _circles.rows();  
	             _elemSize[0] = (int)_circles.elemSize(); // Returns 12 (3 * 4bytes in a float)  
	             _data2[0][0] = new float[_rows[0] * _elemSize[0]/4];  
	             
	             if (_data2[0][0].length>0)
	             {  
	            	 System.out.println("RYn = "+_data2[0][0].length);
	            	 
	               _circles.get(0, 0, _data2[0][0]); // Points to the first element and reads the whole thing  
	                             // into data2  
	               
	                 
	                 Point center= new Point(_data2[0][0][0], _data2[0][0][1]);  
	                  Core.ellipse( webcam_image, center, new Size((double)_data2[0][0][2], (double)_data2[0][0][2]), 0, 0, 360, new Scalar( 255, 0, 255 ), 4, 8, 0 );  
	                t3 = (int)_data2[0][0][0]; t4 = (int)_data2[0][0][1];
	                
	                isB1R = true;
	             }  
	              
	        	
	        ///////////////////////////////
	        	
	     
	          //Core.inRange(distance,new Scalar(0.0), new Scalar(150.0), thresholdedY);  
	          //Core.bitwise_and(thresholded, thresholded2, thresholded);  
	          // Apply the Hough Transform to find the circles  
	          Imgproc.blur(thresholdedY, thresholdedY, new Size(6, 6));  
	        	//Imgproc.Canny(thresholdedY, thresholdedY, 0, 20 * ratio); 
	        	
	        	//_panel1.setimagewithMat(thresholdedY); // _thresholdedY  _webcam_image
	            //_frame1.repaint();
	            
	          Imgproc.HoughCircles(thresholdedY, circles, Imgproc.CV_HOUGH_GRADIENT, 2, thresholdedY.height()/4, 500, detectLim, yRMin, yRMax);   
	         
	          rows[0] = circles.rows();  
	             elemSize[0] = (int)circles.elemSize(); // Returns 12 (3 * 4bytes in a float)  
	             data2[0][1] = new float[rows[0] * elemSize[0]/4];  
	             
	             if (data2[0][1].length>0)
	             {  
	            	 System.out.println("LYn = "+data2[0][1].length);
	               circles.get(0, 0, data2[0][1]); // Points to the first element and reads the whole thing  
	                             // into data2  
	                 
	                 
	                 Point center= new Point(data2[0][1][0], data2[0][1][1]);  
	                  Core.ellipse( webcam_image, center, new Size((double)data2[0][1][2], (double)data2[0][1][2]), 0, 0, 360, new Scalar( 255, 0, 255 ), 4, 8, 0 );  
	                t1 = (int)data2[0][1][0]; t2 = (int)data2[0][1][1];
	                  //System.out.println("_ "+center+" : "+_data2[2]);
	                isB1L = true;
	                
	               
	             
	             }  
	             
	             
	             
	       if(isB1L && isB1R)
	       {
	    	   isB1 = true;
	    	   
	    	   System.out.println("Yellow ");
	  	         Depth.find(t1,t2,t3,t4);
	       }
	       
	       //if(detectCount < 5000)
	       {
	    	   
		       
		       if(Depth.interX > 0 && Depth.interX < 200)// && isB1)
		       {
		    	   
		    	  // AR8.pointCount = nowCount;
		    	   
		    	  // sp[detectCount] = new Sphere( 4, new double[] {900000,0,0}, 5, 5);
		    	  // b1Position[0] = sp[detectCount].center[0] = Depth.interX;
		    	   //b1Position[1] = sp[detectCount].center[1] = 2.5*(Depth.interY - 15);
		    	  // b1Position[2] = sp[detectCount].center[2] = 2.8*Depth.interZ;
		       
			       //sp[detectCount].initSphere(this);
	       			
			       //detectCount++;
		    	   
		    	   b1Position[0] =  Depth.interX;
		    	   b1Position[1] =  2.5*(Depth.interY - 15);
		    	   b1Position[2] =  2.8*Depth.interZ;
	       
			       
		       }
		       else
		    	   isB1 = false;
	       
	       }
	     
	       
	       ///////////////////////////////////////////////////////////2nd ball
	       
	       circles = new Mat();
	       _circles = new Mat();
	       
	   
           //Core.inRange(_distance,new Scalar(0.0), new Scalar(150.0), _thresholdedG);  
           //Core.bitwise_and(_thresholded, _thresholded2, _thresholded); 
           Imgproc.blur(_thresholdedG, _thresholdedG, new Size(6, 6));  
       	//Imgproc.Canny(_thresholdedG, _thresholdedG, 0, 20 * ratio); 
           Imgproc.HoughCircles(_thresholdedG, _circles, Imgproc.CV_HOUGH_GRADIENT, 2, _thresholdedG.height()/4, 500, detectLim, gRMin, gRMax);   
          
            _rows[1] = _circles.rows();  
            _elemSize[1] = (int)_circles.elemSize(); // Returns 12 (3 * 4bytes in a float)  
            _data2[1][0] = new float[_rows[1] * _elemSize[1]/4];  
            
            if (_data2[1][0].length>0)
            {  System.out.println("RGn = ");
              _circles.get(0, 0, _data2[1][0]); // Points to the first element and reads the whole thing  
                            // into data2  
              
                
                Point center= new Point(_data2[1][0][0], _data2[1][0][1]);  
                 Core.ellipse( webcam_image, center, new Size((double)_data2[1][0][2], (double)_data2[1][0][2]), 0, 0, 360, new Scalar( 255, 0, 255 ), 4, 8, 0 );  
               t3 = (int)_data2[1][0][0]; t4 = (int)_data2[1][0][1];
               
               isB2R = true;
            }   
       	
       ///////////////////////////////
       	
          
         //Core.inRange(distance,new Scalar(0.0), new Scalar(150.0), thresholdedG);  
         //Core.bitwise_and(thresholded, thresholded2, thresholded);  
         // Apply the Hough Transform to find the circles  
         Imgproc.blur(thresholdedG, thresholdedG, new Size(6, 6));  
     	//Imgproc.Canny(thresholdedG, thresholdedG, 0, 20 * ratio); 
         Imgproc.HoughCircles(thresholdedG, circles, Imgproc.CV_HOUGH_GRADIENT, 2, thresholdedG.height()/4, 500, detectLim, gRMin, gRMax);   
        
         rows[1] = circles.rows();  
            elemSize[1] = (int)circles.elemSize(); // Returns 12 (3 * 4bytes in a float)  
            data2[1][1] = new float[rows[1] * elemSize[1]/4];  
            
            if (data2[1][1].length>0)
            {  System.out.println("LGn = ");
              circles.get(0, 0, data2[1][1]); // Points to the first element and reads the whole thing  
                            // into data2  
                
                
                Point center= new Point(data2[1][1][0], data2[1][1][1]);  
                 Core.ellipse( webcam_image, center, new Size((double)data2[1][1][2], (double)data2[1][1][2]), 0, 0, 360, new Scalar( 255, 0, 255 ), 4, 8, 0 );  
               t1 = (int)data2[1][1][0]; t2 = (int)data2[1][1][1];
                 //System.out.println("_ "+center+" : "+_data2[2]);
              
              isB2L = true;
              
            }  
            
        // panel1.setimagewithMat(thresholdedG);  // thresholdedG webcam_image
        // frame1.repaint();  
         
         //_panel1.setimagewithMat(thresholdedY); // thresholdedY  webcam_image
        // _frame1.repaint();
	       
	       /////////////////////////////////////////////////////////// 2nd ball end
	       
           if(isB2L && isB2R)
	       {
	    	   isB2 = true;
	    	   
	    	   System.out.println("Green ");
	              Depth.find(t1,t2,t3,t4);
	       }
	   
	       if(detectCount < 5000)
	       {
	    	   
		       
		       if(Depth.interX > 0 && Depth.interX < 200)// && isB2)
		       {
		    	   
	    	  // sp[detectCount] = new Sphere( 4, new double[] {900000,0,0}, 5, 5);
	    	 //  b2Position[0] = sp[detectCount].center[0] = Depth.interX;
	    	 //  b2Position[1] = sp[detectCount].center[1] = 2.5*(Depth.interY - 15);
	    	 //  b2Position[2] = sp[detectCount].center[2] = 2.8*Depth.interZ;
	       
	    	 //  sp[detectCount].initSphere(this);
	       
	    	   //detectCount++;
		    	   
		    	   b2Position[0] =  Depth.interX;
		    	   b2Position[1] =  2.5*(Depth.interY - 15);
		    	   b2Position[2] =  2.8*Depth.interZ;
		       }
		       else
		    	   isB2 = false;
	       
	       }
	  
	    	
	       if(isB1 && isB2)
	       {
	    	   b12Center[0] = (b1Position[0] + b2Position[0]) / 2;
	    	   b12Center[1] = (b1Position[1] + b2Position[1]) / 2;
	    	   b12Center[2] = (b1Position[2] + b2Position[2]) / 2;
	    	   
	    	   System.out.println(Arrays.toString(b1Position));
	    	   System.out.println(Arrays.toString(b2Position));
	    	   System.out.println(Arrays.toString(b12Center));
	    	   
	    	   ballTheta = get_theta(diff(b1Position, b2Position));
	    	   //System.out.println("Theta = "+ballTheta + "Phi = "+ ballPhi);
	    	   
	    	   if(ballTheta>0 && ballTheta<90)
	    	   {
	    		   ballTheta = -(90-ballTheta);
	    	   }
	    	   else if(ballTheta>90 && ballTheta<360)
	    	   {
	    		   ballTheta = ballTheta - 90;
	    	   }
	    		   
	    	   ballPhi = get_phi(diff(b1Position, b2Position));
	    	   
	    	   ballPhi = -(ballPhi - 90);
	    	   
	    	   System.out.println("Theta = "+(int)ballTheta + "Phi = "+ (int)ballPhi);
	    	   
	    	   bodyD = body;
	    	   headTriangleD = headTriangle;
	    	   bladeLiftD = bladeLift;
	    	   tailD = tail;
	    	   bladeRotateD = bladeRotate;
	    	   
	    	   h1 = new Helicopter3D(bodyD, headTriangleD, bladeLiftD, tailD, bladeRotateD);
	    	   h1.initHelicopter3D(this);
	    	   //h1.seek(new double[] {100, 0, 0});
	    	   
	    	   
	    	   
	    	   
	    	   h1.rotateZ(ballTheta, new double[] {0,0,0});
	    	   //h1.rotateX(ballPhi, new double[] {0,0,0});
	    	   h1.seek(b12Center);
	    	   
	    	   trans();
		       repaint();
	    	   
		       
		       b12Center[0] = -b12Center[0];
	    	   b12Center[1] = -b12Center[1];
	    	   b12Center[2] = -b12Center[2];
	    	   h1.seek(b12Center);
	    	   //h1.rotateX(-ballPhi, new double[] {0,0,0});
	    	   h1.rotateZ(-ballTheta, new double[] {0,0,0});
	    	
	       }
	       
	       isB1 = isB2 = isB1L = isB1R = isB2L = isB2R = false;
	       
	     }
	         
	   }

public static float[] HSV(int[] rgb)
{
	float hsv[] = new float[] {-33, -33, -33};
	
	float _r = ((float)rgb[2] / 255);
	float _g = ((float)rgb[1] / 255);
	float _b = ((float)rgb[0] / 255);
	
	float cMin = Math.min(Math.min(_r, _g), _b);
	float cMax = Math.max(Math.max(_r, _g), _b);
	
	//System.out.println("cMin = "+cMin+"cMax = "+cMax);
	
	float delta = (cMax - cMin);
	
	if(cMax != 0)
	{
		hsv[1] = 100 * (delta / cMax);
	}
	else
	{
		hsv[0] = -1;
		hsv[1] = 0;
	}
	
	if(cMax == _r)
	{
		hsv[0] = (60 * (((_g - _b) / delta) % 6));
	}
	else if(cMax == _g)
	{
		hsv[0] = (60 * (((_b - _r) / delta) + 2));
	}
	else if(cMax == _b)
	{
		hsv[0] = (60 * (((_r - _g) / delta) + 4));
	}
	
	if(hsv[0] < 0)
		hsv[0] += 360;
	
	hsv[2] = 100 * cMax;
	
	//if(hsv[1] > 0 && hsv[1] < 20)
	//System.out.println(",,,"+hsv[0]);
	
	return hsv;
}

		
		
	}
	

//##################################################################################################


class Triangle3D	//////////////////////////////////////////////////	TRIANGLE
{
	double[][] triangle3D=new double[3][3];
	int initPoint;
	
	Triangle3D(double[][] triangle3D)
	{
		this.triangle3D = triangle3D;
	}
	
	double[][] initTriangle3D(AR8 ar)
	{
		initPoint = ar.pointCount;
		
		return this.triangle3D;
	}
	
	double[][] getTriangle3D()
	{
		return this.triangle3D;
	}
	
	void seek(double[] move)
	{
		for(int i=0;i<3;i++)
		{
			triangle3D[i][0] += move[0];
			triangle3D[i][0] += move[0];
			triangle3D[i][0] += move[0];
			
			triangle3D[i][1] += move[1];
			triangle3D[i][1] += move[1];
			triangle3D[i][1] += move[1];
			
			triangle3D[i][2] += move[2];
			triangle3D[i][2] += move[2];
			triangle3D[i][2] += move[2];
		}
	}
	
	double[] getCorner(int n)
	{
		return triangle3D[n];
	}
	
	double[] getCOM()
	{
		double[] COM = new double[3];		

		COM[0] = (triangle3D[0][0] + triangle3D[1][0] + triangle3D[2][0])/3;
		COM[1] = (triangle3D[0][1] + triangle3D[1][1] + triangle3D[2][1])/3;
		COM[2] = (triangle3D[0][2] + triangle3D[1][2] + triangle3D[2][2])/3;		
		
		return COM;		
	}
	
	void rotateX(double degree, double[] COM)
	{
		int i;		
		
		double[][] relV = new double[3][3];
		
		for(i=0;i<3;i++)
		{
			relV[i][0] = triangle3D[i][0] - COM[0];
			relV[i][1] = triangle3D[i][1] - COM[1];
			relV[i][2] = triangle3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[3][3];
		
		for(i=0;i<3;i++)
		{
			relNewV[i][1] = relV[i][1]*Math.cos(radian) - relV[i][2]*Math.sin(radian);
			relNewV[i][2] = relV[i][1]*Math.sin(radian) + relV[i][2]*Math.cos(radian);
			relNewV[i][0] = relV[i][0];
		}
		
		for(i=0;i<3;i++)
		{
			triangle3D[i][0] = COM[0] + relNewV[i][0];
			triangle3D[i][1] = COM[1] + relNewV[i][1];
			triangle3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
	
	void rotateY(double degree, double[] COM)
	{
		int i;
		
		double[][] relV = new double[3][3];
		
		for(i=0;i<3;i++)
		{
			relV[i][0] = triangle3D[i][0] - COM[0];
			relV[i][1] = triangle3D[i][1] - COM[1];
			relV[i][2] = triangle3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[3][3];
		
		for(i=0;i<3;i++)
		{
			relNewV[i][2] = relV[i][2]*Math.cos(radian) - relV[i][0]*Math.sin(radian);
			relNewV[i][0] = relV[i][2]*Math.sin(radian) + relV[i][0]*Math.cos(radian);
			relNewV[i][1] = relV[i][1];
		}
		
		for(i=0;i<3;i++)
		{
			triangle3D[i][0] = COM[0] + relNewV[i][0];
			triangle3D[i][1] = COM[1] + relNewV[i][1];
			triangle3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
	
	void rotateZ(double degree, double[] COM)
	{
		int i;		
		
		double[][] relV = new double[3][3];
		
		for(i=0;i<3;i++)
		{
			relV[i][0] = triangle3D[i][0] - COM[0];
			relV[i][1] = triangle3D[i][1] - COM[1];
			relV[i][2] = triangle3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[3][3];
		
		for(i=0;i<3;i++)
		{
			relNewV[i][0] = relV[i][0]*Math.cos(radian) - relV[i][1]*Math.sin(radian);
			relNewV[i][1] = relV[i][0]*Math.sin(radian) + relV[i][1]*Math.cos(radian);
			relNewV[i][2] = relV[i][2];
		}
		
		for(i=0;i<3;i++)
		{
			triangle3D[i][0] = COM[0] + relNewV[i][0];
			triangle3D[i][1] = COM[1] + relNewV[i][1];
			triangle3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
}


class Plane3D	//////////////////////////////////////////////////	PLANE
{
	double[][] plane3D=new double[4][3];
	int initPoint;
	
	Plane3D(double[][] plane3D)
	{
		this.plane3D = plane3D;
	}
	
	double[][] initPlane3D(AR8 ar)
	{
		initPoint = ar.pointCount;
		
		return this.plane3D;
	}
	
	double[][] getPlane3D()
	{
		return this.plane3D;
	}
	
	void seek(double[] move)
	{
		for(int i=0;i<4;i++)
		{
			plane3D[i][0] += move[0];
			plane3D[i][0] += move[0];
			plane3D[i][0] += move[0];
			
			plane3D[i][1] += move[1];
			plane3D[i][1] += move[1];
			plane3D[i][1] += move[1];
			
			plane3D[i][2] += move[2];
			plane3D[i][2] += move[2];
			plane3D[i][2] += move[2];
		}
	}
	
	void put(double[] move)
	{
		for(int i=0;i<4;i++)
		{
			plane3D[i][0] = move[0];
			
			plane3D[i][1] = move[1];
			
			plane3D[i][2] = move[2];
		}
	}
	
	double[] getCorner(int n)
	{
		return plane3D[n];
	}
	
	double[] getCOM()
	{
		double[] COM = new double[3];
		
		COM[0] = (plane3D[0][0] + plane3D[1][0] + plane3D[2][0] + plane3D[3][0])/4;
		COM[1] = (plane3D[0][1] + plane3D[1][1] + plane3D[2][1] + plane3D[3][1])/4;
		COM[2] = (plane3D[0][2] + plane3D[1][2] + plane3D[2][2] + plane3D[3][2])/4;
		
		return COM;
	}
	
	void rotateX(double degree, double[] COM)
	{		
		int i;	
		
		double[][] relV = new double[4][3];
		
		for(i=0;i<4;i++)
		{
			relV[i][0] = plane3D[i][0] - COM[0];
			relV[i][1] = plane3D[i][1] - COM[1];
			relV[i][2] = plane3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[4][3];
		
		for(i=0;i<4;i++)
		{
			relNewV[i][1] = relV[i][1]*Math.cos(radian) - relV[i][2]*Math.sin(radian);
			relNewV[i][2] = relV[i][1]*Math.sin(radian) + relV[i][2]*Math.cos(radian);
			relNewV[i][0] = relV[i][0];
		}
		
		for(i=0;i<4;i++)
		{
			plane3D[i][0] = COM[0] + relNewV[i][0];
			plane3D[i][1] = COM[1] + relNewV[i][1];
			plane3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
	
	void rotateY(double degree, double[] COM)
	{
		int i;		
		
		double[][] relV = new double[4][3];
		
		for(i=0;i<4;i++)
		{
			relV[i][0] = plane3D[i][0] - COM[0];
			relV[i][1] = plane3D[i][1] - COM[1];
			relV[i][2] = plane3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[4][3];
		
		for(i=0;i<4;i++)
		{
			relNewV[i][2] = relV[i][2]*Math.cos(radian) - relV[i][0]*Math.sin(radian);
			relNewV[i][0] = relV[i][2]*Math.sin(radian) + relV[i][0]*Math.cos(radian);
			relNewV[i][1] = relV[i][1];
		}
		
		for(i=0;i<4;i++)
		{
			plane3D[i][0] = COM[0] + relNewV[i][0];
			plane3D[i][1] = COM[1] + relNewV[i][1];
			plane3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
	
	void rotateZ(double degree, double[] COM)
	{
		int i;		
		
		double[][] relV = new double[4][3];
		
		for(i=0;i<4;i++)
		{
			relV[i][0] = plane3D[i][0] - COM[0];
			relV[i][1] = plane3D[i][1] - COM[1];
			relV[i][2] = plane3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[4][3];
		
		for(i=0;i<4;i++)
		{
			relNewV[i][0] = relV[i][0]*Math.cos(radian) - relV[i][1]*Math.sin(radian);
			relNewV[i][1] = relV[i][0]*Math.sin(radian) + relV[i][1]*Math.cos(radian);
			relNewV[i][2] = relV[i][2];
		}
		
		for(i=0;i<4;i++)
		{
			plane3D[i][0] = COM[0] + relNewV[i][0];
			plane3D[i][1] = COM[1] + relNewV[i][1];
			plane3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}

}

class Sphere
{
	Plane3D[][] shell;
	double radii, hInc, vInc;
	double[] center;
	int initPoint,hDiv, vDiv;
	
	Sphere(double radii, double[] center, int horizontalDiv, int verticalDiv)
	{
		this.radii = radii;
		this.center = center;
		this.hDiv = horizontalDiv;
		this.vDiv = verticalDiv;
		shell = new Plane3D[vDiv][hDiv];
	}
	
	void initSphere(AR8 ar)
	{
		
		this.initPoint = ar.pointCount;
		
		hInc = 360 / (double)hDiv;
		vInc = 180 / (double)vDiv;	
		
		for(double phi=0, i=0; i < vDiv; phi = phi + vInc, i++ )
		{
			double upperPhi = phi;
			double lowerPhi = phi - vInc;
			
			for(double theta=0, j=0; j < hDiv; theta = theta + hInc, j++)
			{
				double startTheta = theta;
				double endTheta = theta + hInc;
				
				double[] point1 = new double[3];
				point1[0] = center[0] + radii*Math.cos(Math.toRadians(theta))*Math.sin(Math.toRadians(phi));
				point1[1] = center[1] + radii*Math.sin(Math.toRadians(theta))*Math.sin(Math.toRadians(phi));
				point1[2] = center[2] + radii*Math.cos(Math.toRadians(phi));
				
				double[] point2 = new double[3];
				point2[0] = center[0] + radii*Math.cos(Math.toRadians(theta + hInc))*Math.sin(Math.toRadians(phi));
				point2[1] = center[1] + radii*Math.sin(Math.toRadians(theta + hInc))*Math.sin(Math.toRadians(phi));
				point2[2] = center[2] + radii*Math.cos(Math.toRadians(phi));
				
				double[] point3 = new double[3];
				point3[0] = center[0] + radii*Math.cos(Math.toRadians(theta + hInc))*Math.sin(Math.toRadians(phi + vInc));
				point3[1] = center[1] + radii*Math.sin(Math.toRadians(theta + hInc))*Math.sin(Math.toRadians(phi + vInc));
				point3[2] = center[2] + radii*Math.cos(Math.toRadians(phi + vInc));
				
				double[] point4 = new double[3];
				point4[0] = center[0] + radii*Math.cos(Math.toRadians(theta))*Math.sin(Math.toRadians(phi + vInc));
				point4[1] = center[1] + radii*Math.sin(Math.toRadians(theta))*Math.sin(Math.toRadians(phi + vInc));
				point4[2] = center[2] + radii*Math.cos(Math.toRadians(phi + vInc));
				
				double[][] planeElement = { point1, point2, point3, point4 };
				
				shell[(int) i][(int) j] = new Plane3D(planeElement);
				
				
				
				ar.addPlane(shell[(int) i][(int) j].initPlane3D(ar));
			}
		}
	}	
	
	void seek(double[] move)
	{
		int i,j;
		
		center[0] += 3*move[0];
		center[1] += 3*move[1];
		center[2] += 3*move[2];		//	HOW !???
		
		for(i=0; i < vDiv; i++)		
			for(j=0; j < hDiv; j++)
				shell[i][j].seek(move);
	}
	
	void put(double[] move)
	{
		int i,j;
		
		center[0] = move[0];
		center[1] = move[1];
		center[2] = move[2];		//	HOW !???
		
		for(i=0; i < vDiv; i++)		
			for(j=0; j < hDiv; j++)
				shell[i][j].put(move);
	}
	
	void rotateX(double degree, double[] COM)
	{
		int i,j;
		
		for(i=0; i < vDiv; i++)		
			for(j=0; j < hDiv; j++)
				shell[i][j].rotateX(degree, COM);
	}
	
	void rotateY(double degree, double[] COM)
	{
		int i,j;
		
		for(i=0; i < vDiv; i++)		
			for(j=0; j < hDiv; j++)
				shell[i][j].rotateY(degree, COM);
	}
	
	void rotateZ(double degree, double[] COM)
	{
		int i,j;
		
		for(i=0; i < vDiv; i++)		
			for(j=0; j < hDiv; j++)
				shell[i][j].rotateZ(degree, COM);
	}
}



class Panel extends JPanel{  
  private static final long serialVersionUID = 1L;  
  private BufferedImage image;    
  // Create a constructor method  
  public Panel(){  
    super();  
  }  
  private BufferedImage getimage(){  
    return image;  
  }  
  public void setimage(BufferedImage newimage){  
    image=newimage;  
    return;  
  }  
  public void setimagewithMat(Mat newimage){  
    image=this.matToBufferedImage(newimage);  
    return;  
  }  
  public BufferedImage matToBufferedImage(Mat matrix) {  
    int cols = matrix.cols();  
    int rows = matrix.rows();  
    int elemSize = (int)matrix.elemSize();  
    byte[] data = new byte[cols * rows * elemSize];  
    int type;  
    matrix.get(0, 0, data);  
    switch (matrix.channels()) {  
      case 1:  
        type = BufferedImage.TYPE_BYTE_GRAY;  
        break;  
      case 3:  
        type = BufferedImage.TYPE_3BYTE_BGR;  
       
        byte b;  
        for(int i=0; i<data.length; i=i+3) {  
          b = data[i];  
          data[i] = data[i+2];  
          data[i+2] = b;  
        }  
        break;  
      default:  
        return null;  
    }  
    BufferedImage image2 = new BufferedImage(cols, rows, type);  
    image2.getRaster().setDataElements(0, 0, cols, rows, data);  
    return image2;  
  }  
  @Override  
  protected void paintComponent(Graphics g){  
     super.paintComponent(g);   
     BufferedImage temp=getimage();  
     if( temp != null)
       g.drawImage(temp,10,10,temp.getWidth(),temp.getHeight(), this);  
  }  
}  


class Helicopter3D	//////////////////////////////////////////////////	HELICOPTER
{
	Triangle3D[] headTriangle, bladeLift, bladeRotate;
	Box3D body, tail;
	double inclinationX=0, inclinationY=0, inclinationZ=0;
	double inclinationSpecX=5, inclinationSpecZ=5;
	double bladeRotateMagni = 5;
	
	int initPoint;
	
	Helicopter3D(Box3D body,Triangle3D[] headTriangle, Triangle3D[] bladeLift, Box3D tail, Triangle3D[] bladeRotate)
	{
		this.body=body;
		this.headTriangle=headTriangle;
		this.bladeLift=bladeLift;
		this.tail=tail;
		this.bladeRotate=bladeRotate;		
	}
	
	void initHelicopter3D(AR8 ar)
	{
		this.initPoint = ar.pointCount;
		
		ar.addBox(body.initBox3D(ar));
		
		ar.addTriangle(headTriangle[0].initTriangle3D(ar));
		ar.addTriangle(headTriangle[1].initTriangle3D(ar));
		ar.addTriangle(headTriangle[2].initTriangle3D(ar));
		ar.addTriangle(headTriangle[3].initTriangle3D(ar));
		
		ar.addTriangle(bladeLift[0].initTriangle3D(ar));
		ar.addTriangle(bladeLift[1].initTriangle3D(ar));
		ar.addTriangle(bladeLift[2].initTriangle3D(ar));
		ar.addTriangle(bladeLift[3].initTriangle3D(ar));
		
		ar.addBox(tail.initBox3D(ar));
		
		ar.addTriangle(bladeRotate[0].initTriangle3D(ar));
		ar.addTriangle(bladeRotate[1].initTriangle3D(ar));
		ar.addTriangle(bladeRotate[2].initTriangle3D(ar));
		ar.addTriangle(bladeRotate[3].initTriangle3D(ar));		
	}
	
	void seek(double[] move)
	{
		int i;
		
		body.seek(move);
		
		for(i=0;i<4;i++)	// 4 HeadTriangle		
			headTriangle[i].seek(move);
		
		
		for(i=0;i<4;i++)	// 4 bladeLift		
			bladeLift[i].seek(move);
		
		
		tail.seek(move);
		
		for(i=0;i<4;i++)	// 4 bladeRotate		
			bladeRotate[i].seek(move);
		
	}
	
	void editHelicopter(AR8 ar)
	{
		int i;
		
		ar.editBox(body.getBox3D(), body.initPoint);
		
		for(i=0;i<4;i++)
			ar.editTriangle(headTriangle[i].getTriangle3D(), headTriangle[i].initPoint);
		
		for(i=0;i<4;i++)	
			ar.editTriangle(bladeLift[i].getTriangle3D(), bladeLift[i].initPoint);
		
		ar.editBox(tail.getBox3D(), tail.initPoint);
		
		for(i=0;i<4;i++)
			ar.editTriangle(bladeRotate[i].getTriangle3D(), bladeRotate[i].initPoint);
	
	}
	
	void rotateX(double degree, double[] COM)
	{
		int i;
		
		inclinationX += degree;
		
		if(Math.abs(inclinationX) == 360)
			inclinationX = 0;
	//	inclinationSpecX += degree;
		
		body.rotateX(degree, COM);
		
		for(i=0;i<4;i++)
			headTriangle[i].rotateX(degree, COM);
		
		for(i=0;i<4;i++)
			bladeLift[i].rotateX(degree, COM);
		
		tail.rotateX(degree, COM);
		
		for(i=0;i<4;i++)
			bladeRotate[i].rotateX(degree, COM);
	}
	
	void rotateY(double degree, double[] COM)
	{
		int i;
		
		inclinationY += degree;
		
		if(Math.abs(inclinationY) == 360)
			inclinationY = 0;
		
		body.rotateY(degree, COM);
		
		for(i=0;i<4;i++)
			headTriangle[i].rotateY(degree, COM);
		
		for(i=0;i<4;i++)
			bladeLift[i].rotateY(degree, COM);
		
		tail.rotateY(degree, COM);
		
		for(i=0;i<4;i++)
			bladeRotate[i].rotateY(degree, COM);
	}
	
	void rotateZ(double degree, double[] COM)
	{
		int i;
		
		inclinationZ += degree;
		
		if(Math.abs(inclinationZ) == 360)
			inclinationZ = 0;
		//inclinationSpecZ += degree;
		
		body.rotateZ(degree, COM);
		
		for(i=0;i<4;i++)
			headTriangle[i].rotateZ(degree, COM);
		
		for(i=0;i<4;i++)
			bladeLift[i].rotateZ(degree, COM);
		
		tail.rotateZ(degree, COM);
		
		for(i=0;i<4;i++)
			bladeRotate[i].rotateZ(degree, COM);
	}
	
	void circulateLift()	//	Z
	{
		for(int i=0;i<4;i++)
		{
			bladeLift[i].rotateX(-inclinationX, bladeLift[0].getCorner(0));
			bladeLift[i].rotateY(-inclinationY, bladeLift[0].getCorner(0));
			bladeLift[i].rotateZ(-inclinationZ, bladeLift[0].getCorner(0));
			
			bladeLift[i].rotateZ(inclinationSpecZ, bladeLift[0].getCorner(0));
			
			bladeLift[i].rotateZ(inclinationZ, bladeLift[0].getCorner(0));
			bladeLift[i].rotateY(inclinationY, bladeLift[0].getCorner(0));
			bladeLift[i].rotateX(inclinationX, bladeLift[0].getCorner(0));	
		}		
	}
	
	void circulateRotate()	//	X
	{
		for(int i=0;i<4;i++)
		{
			bladeRotate[i].rotateX(-inclinationX, bladeRotate[0].getCorner(0));
			bladeRotate[i].rotateY(-inclinationY, bladeRotate[0].getCorner(0));
			bladeRotate[i].rotateZ(-inclinationZ, bladeRotate[0].getCorner(0));
			
			bladeRotate[i].rotateX(inclinationSpecX, bladeRotate[0].getCorner(0));
			
			bladeRotate[i].rotateZ(inclinationZ, bladeRotate[0].getCorner(0));
			bladeRotate[i].rotateY(inclinationY, bladeRotate[0].getCorner(0));
			bladeRotate[i].rotateX(inclinationX, bladeRotate[0].getCorner(0));
		}
	}
	
	void circulateLiftRecover()	//	Z
	{
		
		for(int i=0;i<4;i++)
		{
			
			bladeLift[i].rotateX((inclinationSpecZ*inclinationX)/(360), bladeLift[0].getCorner(0));
			bladeLift[i].rotateY(inclinationY, bladeLift[0].getCorner(0));
			bladeLift[i].rotateZ(inclinationSpecZ, bladeLift[0].getCorner(0));
		}
	}
	
	void circulateRotateRecover()	//	X
	{
		for(int i=0;i<4;i++)
		{
			bladeRotate[i].rotateX(inclinationSpecX, bladeRotate[0].getCorner(0));
			bladeRotate[i].rotateY(-inclinationY, bladeRotate[0].getCorner(0));
			bladeRotate[i].rotateZ(-inclinationZ, bladeRotate[0].getCorner(0));
		}
	}
}




class Box3D	//////////////////////////////////////////////////	BOX
{
	double[][] box3D=new double[8][3];
	int initPoint;
	
	Box3D(double[][] box3D)
	{
		this.box3D = box3D;
	}
	
	double[][] initBox3D(AR8 ar)
	{
		initPoint = ar.pointCount;
		
		return this.box3D;
	}
	
	double[][] getBox3D()
	{
		return this.box3D;
	}
	
	void seek(double[] move)
	{
		for(int i=0;i<8;i++)
		{
			box3D[i][0] += move[0];
			box3D[i][0] += move[0];
			box3D[i][0] += move[0];
			
			box3D[i][1] += move[1];
			box3D[i][1] += move[1];
			box3D[i][1] += move[1];
			
			box3D[i][2] += move[2];
			box3D[i][2] += move[2];
			box3D[i][2] += move[2];
		}
	}
	
	double[] getCorner(int n)
	{
		return box3D[n];
	}
	
	double[] getCOM()
	{
		double[] COM = new double[3];
		
		COM[0] = (box3D[0][0] + box3D[1][0] + box3D[2][0] + box3D[3][0] + box3D[4][0] + box3D[5][0] + box3D[6][0] + box3D[7][0])/8;
		COM[1] = (box3D[0][1] + box3D[1][1] + box3D[2][1] + box3D[3][1] + box3D[4][1] + box3D[5][1] + box3D[6][1] + box3D[7][1])/8;
		COM[2] = (box3D[0][2] + box3D[1][2] + box3D[2][2] + box3D[3][2] + box3D[4][2] + box3D[5][2] + box3D[6][2] + box3D[7][2])/8;		
		
		return COM;
	}
	
	void rotateX(double degree, double[] COM)
	{
		int i;		
		
		double[][] relV = new double[8][3];
		
		for(i=0;i<8;i++)
		{
			relV[i][0] = box3D[i][0] - COM[0];
			relV[i][1] = box3D[i][1] - COM[1];
			relV[i][2] = box3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[8][3];
		
		for(i=0;i<8;i++)
		{
			relNewV[i][1] = relV[i][1]*Math.cos(radian) - relV[i][2]*Math.sin(radian);
			relNewV[i][2] = relV[i][1]*Math.sin(radian) + relV[i][2]*Math.cos(radian);
			relNewV[i][0] = relV[i][0];
		}
		
		for(i=0;i<8;i++)
		{
			box3D[i][0] = COM[0] + relNewV[i][0];
			box3D[i][1] = COM[1] + relNewV[i][1];
			box3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
	
	void rotateY(double degree, double[] COM)
	{
		int i;		
		
		double[][] relV = new double[8][3];
		
		for(i=0;i<8;i++)
		{
			relV[i][0] = box3D[i][0] - COM[0];
			relV[i][1] = box3D[i][1] - COM[1];
			relV[i][2] = box3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[8][3];
		
		for(i=0;i<8;i++)
		{
			relNewV[i][2] = relV[i][2]*Math.cos(radian) - relV[i][0]*Math.sin(radian);
			relNewV[i][0] = relV[i][2]*Math.sin(radian) + relV[i][0]*Math.cos(radian);
			relNewV[i][1] = relV[i][1];
		}
		
		for(i=0;i<8;i++)
		{
			box3D[i][0] = COM[0] + relNewV[i][0];
			box3D[i][1] = COM[1] + relNewV[i][1];
			box3D[i][2] = COM[2] + relNewV[i][2];
		}				
	}
	
	void rotateZ(double degree, double[] COM)
	{		
		int i;		
			
		double[][] relV = new double[8][3];
		
		for(i=0;i<8;i++)
		{
			relV[i][0] = box3D[i][0] - COM[0];
			relV[i][1] = box3D[i][1] - COM[1];
			relV[i][2] = box3D[i][2] - COM[2];
		}
		
		double radian = Math.toRadians(degree);
		
		double[][] relNewV = new double[8][3];
		
		for(i=0;i<8;i++)
		{
			relNewV[i][0] = relV[i][0]*Math.cos(radian) - relV[i][1]*Math.sin(radian);
			relNewV[i][1] = relV[i][0]*Math.sin(radian) + relV[i][1]*Math.cos(radian);
			relNewV[i][2] = relV[i][2];
		}
		
		for(i=0;i<8;i++)
		{
			box3D[i][0] = COM[0] + relNewV[i][0];
			box3D[i][1] = COM[1] + relNewV[i][1];
			box3D[i][2] = COM[2] + relNewV[i][2];
		}
				
	}
}

class Depth {  
	 
	 static double xL, xR, yL, yR;
	 static double width = 30;
	 static double thetaL, thetaR, thetaV;
	 public static double interX, interY, interZ;
	 static double mL, mR;
	 static double temp;
	 
	 
	 public static void find(int XL, int YL, int XR, int YR)
	 {
		 xL = XL - 640; xR = XR - 640;
		 yL = 480 - YL; yR = 480 - YR;
		 
		 thetaV = 21*(yL/480);
		 thetaL = -30*(xL/640);
		 thetaR = -30*(xR/640);
		 
		 mL = Math.tan(Math.toRadians(thetaL));
		 mR = Math.tan(Math.toRadians(thetaR));
		 
		 interX = 1/((((mL + mR) / 2) - mL) * 2 / width);
		 interY = 1*mL*interX + width/2;
		 interZ = 1*interX * Math.tan(Math.toRadians(thetaV));
		 
		 
		 System.out.println("X = "+(int)interX+", Y = "+(int)interY+", Z = "+(int)interZ);
	 }
	 
  
}