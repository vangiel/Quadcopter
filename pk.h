#define P_LEN 40

class tPaquete {
  unsigned char por ;
  unsigned char cks ;
public :
  unsigned char len ;
  enum { VACIO , INICIADO , LEE , LLENO } estado ;
  unsigned char datos[P_LEN] ;
  tPaquete( ) {
     estado = VACIO ;
  } ;
  bool nuevoDato( unsigned char dato ) {
    // devuelve si se ha leido un dato o no 
    switch( estado ) {
      case VACIO :
        if( dato == 0x16 ) {
          datos[ 0 ] = cks = dato ; 
          por = 1 ;
          len = 2 ; // ancho del paquete 
          estado = INICIADO ;     
        } 
		break ;
      case INICIADO :
		datos[ por++ ] = dato ;
		cks = cks + dato ;
		estado = LEE ;
        switch( dato ) {
          case 0x00 : // Escribir un 0
          case 0x01 : // Escribir un 1
            len = 4 ;
            break ;
          case 0x02 : // envio de muestra
            len = 5 ;
            break ;
          case 0x03 : // paquete con los datos del acelerometro
            len = 17 ;
            break ;
          case 0x04 : // paquete con los datos de los motores 
            len = 3+2*4 /* 4 muestras de 16bits */ ;
            break ;
          case 0x05 : // envio de ack
            len = 3 ;
            break ;
		  case 0x06 : // limite,kp,ki,kd
		    len = 3 + 8*4 ;
			break ;
          default :
            estado = VACIO ;
        }
        break ;
      case LEE :
        datos[por++] = dato ;  
        if( por == len ) {
          if( dato == cks ) {
            estado = LLENO ;
            return true ;
          }
        } else {
          cks = cks + dato ;
        }
        break ;
      case LLENO :
        return true ;
    }
    return false ;
  }
  bool inicia( unsigned char comando ) {
    estado = LLENO ;
    datos[ 0 ] = 0x16 ;
    datos[ por = 1 ] = comando ;
    switch( comando ) {
      case 0x00 :
      case 0x01 :
        len = 4 ;
        break ;
      case 0x02 :
        len = 5 ;
        break ;
      case 0x03 : 
        len = 17 ;
        break ;
      case 0x04 : // paquete con los datos de los motores 
        len = 3+2*4 /* 4 muestras de 16bits */ ;
        break ;
      case 0x05 : 
        len = 3 ; // ACK
        break ;
      case 0x06 : // limite,kp,ki,kd
	len = 3 + 8*4 ;
	break ;
      default :
        return false ; 
    }
    datos[len-1] = 0 ; // el checksum
    return true ;
  }
  void finaliza( ) {
    cks = 0 ;
    for(  int f = len-2 ; f >= 0 ; f-- )
      cks = cks + datos[ f ] ;
    datos[ len-1 ] = cks ;
  }
} ;

int miscanf( const char * c , float &f ) ;  

extern tPaquete pk ;

extern float pk_limite ;
extern float pk_kp ;
extern float pk_ki ;
extern float pk_kd ;

