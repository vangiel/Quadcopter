
#include "pk.h"
#include "arduino.h"

tPaquete pk ;

void serialEvent( ) {
#define DBG_
#ifdef DBG
  char a ;
  SEntrada.nuevoDato( a = Serial.read( ) ) ;
  Serial.println( (int)a ) ; // eco para depurar
#else
  pk.nuevoDato( Serial.read( ) ) ;
#endif
}

float pk_limite ;
float pk_kp ;
float pk_ki ;
float pk_kd ;

int miscanf( const char * c , float & f ) {
  f = 0 ; double decimal ; double signo = 1 ;
  enum { ENTERO , DECIMAL } estado = ENTERO ;
  const char * por = c ;
  switch( *por ) {
    case '+' : por++ ; break ;
    case '-' : signo = -1 ; por++ ; break ;
  }
  for( ; *por != 0 ; por++ ) {
    if( estado == ENTERO ) {
      switch( *por ) { 
        case '.' :
           estado = DECIMAL ; decimal = 10 ; break ;
        default :
            if( *por >= '0' && *por <= '9' ) 
                f = f * 10 + (*por - '0') ;
            else
                return false ;
      }
    } else {      
      if( *por >= '0' && *por <= '9' ) {
          f = f +(((double) (*por-'0'))/decimal) ;
      } else 
          return false ;      
      decimal = decimal * 10 ;
    }
  }
  f = signo * f ;
  return true ;
}
