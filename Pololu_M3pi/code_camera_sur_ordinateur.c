#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#define MIN3(a, b, c) ( (a) < (b) ? ( (a) < (c) ? (a) : ( (b) < (c) ? (b) : (c) ) ) : ( (b) < (c) ? (b) : (c) ) )
#define MAX3(a, b, c) ( (a) > (b) ? ( (a) > (c) ? (a) : ( (b) > (c) ? (b) : (c) ) ) : ( (b) > (c) ? (b) : (c) ) )

typedef struct fvec4_t fvec4_t;
struct fvec4_t {
  float * data;
  int nelem;
  int curelem;
  int maxelem;
};


float calculAngle( float  dist_ob_bl ,float  dist_ros_bl,float  dist_ob_ros)
{
    float angle  = acosf(((pow(dist_ob_bl,2) + pow(dist_ros_bl,2)) - pow(dist_ob_ros,2)) / (2*(dist_ob_bl*dist_ros_bl))) *180/M_PI; 
    return (angle);
}

static fvec4_t * new_fvec4(int nelem) {
  fvec4_t * v = malloc(sizeof *v);
  assert(v);
  v->nelem = nelem;
  v->curelem = v->maxelem = 0;
  v->data = malloc(4 * v->nelem * sizeof *v->data);
  assert(v->data);
  return v;
}

static void free_fvec4(fvec4_t * v) {
  if(v) {
    if(v->data)
      free(v->data);
    free(v);
  }
}

static void fill_fvec4(fvec4_t * v, float *vec3) {
  int i;
  if(v->curelem == v->nelem) {
    v->curelem = 0;
  }
  for(i = 0; i < 3; i++)
    v->data[4 * v->curelem + i] = vec3[i];
    v->data[4 * v->curelem + 3] = 1;
    if(v->maxelem < ++v->curelem)
    v->maxelem = v->curelem;
}

int compar(const void * p0, const void * p1) {
  const float * f0 = p0, * f1 = p1;
  return f0[3] < f1[3] ? 1 : -1;
}

static void clusterize_fvec4(fvec4_t * v, fvec4_t * cv, float thresholdd) {
  int ii, i, j, nearesti;
  float nearestd, d, dx ,dy, dr;
  cv->curelem = cv->maxelem = cv->maxelem >= 2 ? 2 : 0;
  for(ii = 0; ii < cv->maxelem; ii++)
    cv->data[4 * ii + 3] = 0.0;
  for(ii = 0; ii < v->maxelem; ii++) {
    i = (v->curelem + ii) % v->nelem;
    for(j = 0, nearestd = FLT_MAX, nearesti = -1; j < cv->maxelem; j++) {
      dx = v->data[4 * i] - cv->data[4 * j];
      dy = v->data[4 * i + 1] - cv->data[4 * j + 1];
      dr = v->data[4 * i + 2] - cv->data[4 * j + 2];
      d = sqrt(dx * dx + dy * dy + dr * dr);
      if(nearestd > d) {
	nearestd = d;
	nearesti = j;
      }
    }
    if(nearestd < thresholdd) {
      for(j = 0; j < 3; j++)
	cv->data[4 * nearesti + j] = (v->data[4 * i + j] + cv->data[4 * nearesti + j]) / 2.0;
      cv->data[4 * nearesti + 3] += thresholdd / (nearestd + 0.1);
    } else {
      memcpy(&(cv->data[4 * cv->curelem]), &(v->data[4 * i]), 4 * sizeof *cv->data);
      cv->maxelem = ++cv->curelem;
      if(cv->maxelem >= cv->nelem)
	break;
    }
  }
  qsort(cv->data, cv->nelem, 4 * sizeof *cv->data, compar);
}

static void rgb2hsv(float r, float g, float b, float * h, float * s, float * v) {
  float min = MIN3(r, g, b), max = MAX3(r, g, b);
  *v = max;
  *s = (max != 0.0) ? ((max - min) / max) : 0.0;
  if(*s == 0.0)
    *h = 1.0;
  else {
    float delta = 1.0 / (max - min);
    if(r == max)
      *h = delta * (g - b) ;
    else if(g == max)
      *h = 2.0 + delta * (b - r);
    else if(b == max)
      *h = 4.0 + delta * (r - g);
    *h = (*h < 0.0) ? (*h / 6.0 + 1.0) : (*h / 6.0);
  }
}

/*La fonction qui permet de garder la couleur bleu */
static void garderLeBleu(IplImage * gb) {
  int i, j;
  unsigned char * dd = (unsigned char *)gb->imageData;
  float r, g, b, h, s, v;
  assert(gb->nChannels == 3);
  for(i = 0; i < gb->height; i++) {
    for(j = 0; j < gb->width; j++) {
      b = dd[i * gb->widthStep + j * gb->nChannels + 0] / 255.0;
      g = dd[i * gb->widthStep + j * gb->nChannels + 1] / 255.0;
      r = dd[i * gb->widthStep + j * gb->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
    if (h >= 0.4 && h <= 0.7  && v >= 0.5 && v <= 0.9 &&  s >= 0.2 && s <=1)
	dd[i * gb->widthStep + j * gb->nChannels + 0] = dd[i * gb->widthStep + j * gb->nChannels + 1] = dd[i * gb->widthStep + j * gb->nChannels + 2] =255;
      else
	dd[i * gb->widthStep + j * gb->nChannels + 0] = dd[i * gb->widthStep + j * gb->nChannels + 1] = dd[i * gb->widthStep + j * gb->nChannels + 2] = 0;
    }
  }
}

/*La fonction qui permet de garder la couleur */
static void garderLeVert(IplImage * img) {
  int i, j;
  unsigned char * dd = (unsigned char *)img->imageData;
  float r, g, b, h, s, v;
  assert(img->nChannels == 3);
  for(i = 0; i < img->height; i++) {
    for(j = 0; j < img->width; j++) {
      r = dd[i * img->widthStep + j * img->nChannels + 0] / 255.0;
      g = dd[i * img->widthStep + j * img->nChannels + 1] / 255.0;
      b = dd[i * img->widthStep + j * img->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
      if(h >= 0.1 && h <= 0.5 && v>=0.3 && v <= 0.8 &&  s >= 0.3 && s <= 0.5)
    dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 255;
      else
    dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 0;
    }
  }
}

/*La fonction qui permet de garder la couleur jaune (l'objectif)*/
static void trouverLobjectif(IplImage * img) {
  int i, j;
  unsigned char * dd = (unsigned char *)img->imageData;
  float r, g, b, h, s, v;
  assert(img->nChannels == 3);
  for(i = 0; i < img->height; i++) {
    for(j = 0; j < img->width; j++) {
      b = dd[i * img->widthStep + j * img->nChannels + 0] / 255.0;
      g = dd[i * img->widthStep + j * img->nChannels + 1] / 255.0;
      r = dd[i * img->widthStep + j * img->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
      if(h >=0 && h <=0.2 && v >= 0.7 && v <= 1 &&  s >= 0 && s <= 1)
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 255;
      else
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 0;
    }
  }
}

/*La fonction qui permet de garder la couleur rose*/
static void garderLeRose(IplImage * img) {
  int i, j;
  unsigned char * dd = (unsigned char *)img->imageData;
  float r, g, b, h, s, v;
  assert(img->nChannels == 3);
  for(i = 0; i < img->height; i++) {
    for(j = 0; j < img->width; j++) {
      b = dd[i * img->widthStep + j * img->nChannels + 0] / 255.0;
      g = dd[i * img->widthStep + j * img->nChannels + 1] / 255.0;
      r = dd[i * img->widthStep + j * img->nChannels + 2] / 255.0;
      rgb2hsv(r, g, b, &h, &s, &v);
      if(h >=0.7 && h <=1 && v >= 0.6 && v <= 1 &&  s >= 0.3 && s <= 1)
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 255;//s * 255.0;
      else
	dd[i * img->widthStep + j * img->nChannels + 0] = dd[i * img->widthStep + j * img->nChannels + 1] = dd[i * img->widthStep + j * img->nChannels + 2] = 0;
    }
  }
}

/*La fonction qui envoie la trame au robot */
 void message(char *trans)
 {
     printf("\nla trame est :%s\n",trans);
     printf("\n");
     int  fd = open("/dev/ttyACM0", O_RDWR);

     if (fd == -1){
         fprintf(stderr, "Unable to open the device\n");
         exit(0);
     }
     write(fd, trans, sizeof trans);
  }


/*================La fonction Main()=============*/

int main(int argc, char ** argv) {
/*  creation des variables qui vont contenir les images*/
    IplImage* img = NULL, * imgG = NULL;
    IplImage* gr = NULL;
    IplImage* gb = NULL;
    IplImage* gx = NULL ;
    IplImage* descer = NULL ;
    IplImage* obj = NULL ;
    CvCapture* capture = NULL;
    CvSize s;
    int i, j;
    int fd;
    CvMemStorage * memSto = cvCreateMemStorage(0);
    CvSeq * res = NULL;
    CvSeq * bleu_res = NULL;
    CvSeq * rose_res = NULL;
    CvSeq * jaune_res = NULL;
    fvec4_t * v = new_fvec4(100), * cv = new_fvec4(100);
    fvec4_t * b = new_fvec4(100), * cb = new_fvec4(100);
    
    /*l'activation du webcam*/
    if(!(capture = cvCaptureFromCAM(1)))
      capture = cvCaptureFromCAM(CV_CAP_ANY);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 640);
    cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 480);
    s.width = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH);
    s.height = (int)cvGetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT);
    fprintf(stderr, "dimensions du flux %dx%d\n", s.width, s.height);
    
    /*Creation des fenêtres pour afficher les images*/
    cvNamedWindow("Garder_vert", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Garder_bleu", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Garder_rose", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Original", CV_WINDOW_AUTOSIZE);
    cvNamedWindow("Objectif", CV_WINDOW_AUTOSIZE);
    imgG = cvCreateImage(s, IPL_DEPTH_8U, 1);
    
    /* concersion des images de 16 bits en 8 bites */ 
    gx= cvCreateImage(s,IPL_DEPTH_8S , 1 );
    gb= cvCreateImage(s,IPL_DEPTH_8S , 1 );
    gr= cvCreateImage(s,IPL_DEPTH_8S , 1 );
    obj= cvCreateImage(s,IPL_DEPTH_8S , 1 );
    descer= cvCreateImage(s,IPL_DEPTH_8S , 1 );
    
	/*declaration les coordonnees des cercles qui permetront de detecter
	  tous les objets circulaire qui seront utilisés */
	float coodBx=0.0,coodBy=0.0,coodV2x = 0.0,coodV2y = 0.0;
	float coodRx=0.0,coodRy=0.0,coodObx=0.0,coodOby=0.0, coodVx=0.0,coodVy=0.0;
	int  dist_ob_bl = 0;
	float  dist_ros_bl = 0.0;
	float  dist_ob_ros = 0.0;
	float  angl_en_bl = 0.0;
	float  dist_ob_vt = 0.0;
	/*conersion de pixel en centimetre en fonction de la hauter de la camera (ici 2m du sol)*/
	float cm =0.202020202;
	
    CvPoint positionBleu;
    CvPoint posVert;
    CvPoint positionRose;
    CvPoint positionObjet;
    double delay=0.0;
    time_t start, now;
    time(&start);
    
    /*la boucle infini qui permet de capturer les images
      avec la webcame jusqu'a ce que le robot atteint 
      l'objectif */
    while((img = cvQueryFrame(capture))) {
    cvSmooth(img, img, CV_GAUSSIAN, 5, 5, 0, 0);
    
    /* cloner l'image originale */
    descer = cvCloneImage(img);
    
    /* traitement de l'image pour garder la couleur verte et ces coordonnees*/
    gx = cvCloneImage(img);
    garderLeVert(gx);
    cvCvtColor(gx, imgG, CV_RGB2GRAY);
    res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 30, 2, 20);
    for(i = 0; i < res->total; i++) {
      float * p = (float *)cvGetSeqElem(res, i);
      CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
      posVert = cvPoint(cvRound(p[0]), cvRound(p[1]));
      cvCircle(gx, pt, cvRound(p[2]), CV_RGB(0xff, 0, 0), 0, 8, 0);
      fill_fvec4(v, p);
      clusterize_fvec4(v, cv, 10);
      if(cv->maxelem >= 2) {
         for(j = 0; j < 2; j++) {
            pt = cvPoint(cvRound(cv->data[4 * j]), cvRound(cv->data[4 * j + 1]));
            if(j==0){
               coodVx = pt.x; 
               coodVy = pt.y;
            }
            else
            {
              coodV2x = pt.x; 
              coodV2y = pt.y;
            }
         }
       }
       cvCircle(descer, posVert, cvRound(2), CV_RGB(0xff, 0 , 0), 1, 8, 0);
      } 
    
    /*
    garderLeVert(gx);
    cvCvtColor(gx, imgG, CV_RGB2GRAY);
    res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 30, 2, 20);
    for(i = 0; i < res->total; i++) {
      float * p = (float *)cvGetSeqElem(res, i);
      CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
      CvPoint  posVert = cvPoint(cvRound(p[0]), cvRound(p[1]));
      cvCircle(gx, pt, cvRound(p[2]), CV_RGB(0, 0xff, 0), 0, 8, 0);
      coodVx =p[0];
      coodVy =p[1];
       fill_fvec4(v, p);
      clusterize_fvec4(b, cb, 10);
      if(cb->maxelem >= 1) {
	     for(j = 0; j < 2; j++) {
	     pt = cvPoint(cvRound(cb->data[4 * j]), cvRound(cb->data[4 * j + 1]));
	     cvCircle(gb, pt, cvRound(MIN(100.0, fabs(cb->data[4 * j + 2]))), CV_RGB(0, 0xff, 0), 1, 8, 0);
	     }
      }
      cvCircle(descer, posVert, cvRound(2), CV_RGB(0, 0 , 0xff), 1, 8, 0);
     // printf("les coodonnées du vert(%0.2f , %0.2f)\n", p[0],p[1]);
    }
    */
    
    /* traitement de l'image pour garder la couleur bleu et ces coordonnees*/
    gb = cvCloneImage(img);
    garderLeBleu(gb);
    cvCvtColor(gb, imgG, CV_RGB2GRAY);
    bleu_res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 20, 2, 20);
    for(i = 0; i < bleu_res->total; i++) {
       float * p1 = (float *)cvGetSeqElem(bleu_res, i);
       positionBleu = cvPoint(cvRound(p1[0]), cvRound(p1[1]));
       cvCircle(gb, positionBleu, cvRound(p1[2]), CV_RGB(0, 0 , 0xff), 2, 8, 0);
       cvCircle(descer, positionBleu, cvRound(2), CV_RGB(0, 0 , 0xff), 0, 8, 0);
       coodBx =positionBleu.x;
       coodBy =positionBleu.y;
    }
    
    
    /* traitement de l'image pour garder la couleur rose et ces coordonnees*/
    gr = cvCloneImage(img);
    garderLeRose(gr);
    cvCvtColor(gr, imgG, CV_RGB2GRAY);
    rose_res = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 25, 2, 20);
    for(i = 0; i < rose_res->total; i++) {
      float * p2 = (float *)cvGetSeqElem(rose_res, i);
       positionRose= cvPoint(cvRound(p2[0]), cvRound(p2[1]));
       cvCircle(gr, positionRose, cvRound(p2[2]), CV_RGB(0xff, 0 , 0), 0, 8, 0);
       coodRx = positionRose.x;
       coodRy = positionRose.y;
       cvCircle(descer, positionRose, cvRound(2), CV_RGB(0, 0 , 0xff), 0, 8, 0);
   }
   
   /*detecter l'objectif en fonction de la couleur qui est Jaune */
    obj = cvCloneImage(img);
    trouverLobjectif(obj);
    cvCvtColor(obj, imgG, CV_RGB2GRAY);
    
   jaune_res  = cvHoughCircles(imgG, memSto, CV_HOUGH_GRADIENT, 2, imgG->width / 4, 80, 40, 2, 20);
    for(i = 0; i < jaune_res ->total; i++) {
      float * po = (float *)cvGetSeqElem(jaune_res , i);
      positionObjet = cvPoint(cvRound(po[0]), cvRound(po[1]));
      cvCircle(gr, positionObjet, cvRound(po[2]), CV_RGB(0, 0xff , 0), 1, 8, 0);
      coodObx =positionObjet.x;
      coodOby =positionObjet.y;
      /* dessiner un point sur l'objet */
      cvCircle(descer, positionObjet , cvRound(4), CV_RGB(0, 0xff , 0), 2, 8, 0);
    }
    /* tracer la ligne */
     cvLine(descer, positionObjet, positionBleu, CV_RGB(0, 0 ,0xff), 2, 8, 0 );
     
    /*calcule de distance entre les differents objets detectés*/
    /*objet -> bleu*/
     dist_ob_bl = sqrt(pow((coodBx-coodObx ),2) + pow((coodBy-coodOby ),2)) *cm;
     dist_ob_ros = sqrt(pow((coodRx-coodObx ),2) + pow((coodRy-coodOby ),2)) *cm;
     dist_ros_bl = sqrt(pow((coodRx-coodBx ),2) + pow((coodRy-coodBy ),2)) *cm;
     
     /*objet -> vert1*/
     dist_ob_vt = sqrt(pow((coodVx-coodObx ),2) + pow((coodVy-coodOby ),2)) *cm;
     
     /*rose -> vert*/
     float dist_ros_vt = sqrt(pow((coodRx-coodVx ),2) + pow((coodRy-coodVy ),2)) *cm;
     float differ_vtO_blO=0.0;
     
     if(dist_ob_vt != 0.0 && dist_ob_bl != 0.0 )
     {
         differ_vtO_blO =  ( dist_ob_vt - dist_ob_bl);
     }
    
    /* calcul d'angle*/
    float angle = calculAngle(dist_ob_bl , dist_ros_bl, dist_ob_ros) ;
     
    angl_en_bl = (angle-90);
    int abs_ang = (int)fabs(angl_en_bl);
    printf("L'angle est:  (%0.2f)\n",abs_ang);
    
    /* Creation de la trame qui sera envoyée au robot */
    printf("la distance rose vert : %0.2f", dist_ros_vt);
    char convert[4];
    int val;
    /*calcul de temps d'attente avant d'envoyer une trame au robot*/
    time(&now);
    delay = difftime(now, start);
    printf("\n le delay avant est: %f\n",delay);
        
   if(delay > 3.0)
    {
           printf("\n le delay  apres est: %f\n",delay);
          if( abs_ang < 12 && differ_vtO_blO < 0.0 && dist_ros_vt < 4.30)
          {
            dist_ob_bl -=4;
            /*concatenation de la lettre et la distance ou l'angle a parcourir par le robot */
            sprintf(convert,"A%d",dist_ob_bl);
            message(convert);
            
          }
          
          if( abs_ang < 12 && differ_vtO_blO > 0.0 && dist_ros_vt < 4.30)
          {
           val = 180;
           sprintf(convert,"G%d",val);
           message(convert);
          
          }
          if( abs_ang > 12 && angl_en_bl < 0.0 && dist_ros_vt < 4.30)
          {
            sprintf(convert,"G%d",abs_ang);
            message(convert);
           
          }
          if( abs_ang > 12 && angl_en_bl > 0.0 && dist_ros_vt < 4.30)
          {
             sprintf(convert,"D%d",abs_ang);
             message(convert);
            
          }
          
         delay =0.0;
         time(&start);
    }
       
    printf("\n");
    printf("Difference Bleu->OB et vert->OB :  (%0.2f)\n",differ_vtO_blO);
    printf("\n\n");
    
    /*Le condition de fin lorsque le robot atteint l'objectif */ 
    if(dist_ob_bl < 15.0 && dist_ob_bl > 0.0)
    {
        sprintf(convert,"M%d",dist_ob_bl);
        message(convert); 
        exit(0);  
    }
    
     /*Affichage des images créées.
      j'ai mis les autres fenêtre en 
      commentaire et j'ai laisser que 
      l'image originale
     */
     
   // cvShowImage("Garder_vert", gx);
    //cvShowImage("Objectif", obj);
    //cvShowImage("Garder_bleu", gb);
   // cvShowImage("Garder_rose", gr);
   cvShowImage("Original", descer);
   if( (cvWaitKey(10) & 0xFF) == 27 )
   break;
  }

  if(capture)
    cvReleaseCapture(&capture);
  free_fvec4(v);
  free_fvec4(cv);
  return 0;
}


