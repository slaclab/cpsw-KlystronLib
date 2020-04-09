#include <cpsw_api_user.h>
#include <KlystronFw.h>
#include <vector>
#include <string>
#include <dlfcn.h>

#include <getopt.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <fstream>

int
main(int argc, char **argv)
{
const char *ip_addr = "127.0.0.1";
int         opt;
const char *use_yaml = 0;
const char *dmp_yaml = 0;
const char *cfg_yaml = 0;
const char *save_stream = 0;


        while ( (opt = getopt(argc, argv, "y:d:c:s:")) > 0 ) {
                switch (opt) {
                        default:
                                fprintf(stderr,"Unknown option -'%c'\n", opt);
                        case 'y': use_yaml    = optarg;   break;
                        case 'c': cfg_yaml    = optarg;   break;
                        case 'd': dmp_yaml    = optarg;   break;
                        case 's': save_stream = optarg;   break;
                }
        }

try {
	Path p = IPath::loadYamlFile( use_yaml, "NetIODev" );
        Path p_mmio = p->findByName("mmio");

	KlystronFw fw = IKlystronFw::create( p_mmio );

/* for now we will handle streams ourselves */
	std::vector<Stream> strms;
	for ( int i = 0; i < 8; i++ )
	{
		std::stringstream num;
		std::string str("Stream");
		num << str << i;
		
		strms.push_back( IStream::create( p->findByName(num.str().c_str()) ) );
	}

/* To load default config */
	if ( cfg_yaml ) {
		fw->loadConfigFromYamlFile( cfg_yaml );
	}

/* Exercise the FW API */
	fw->OutputEnable( false );   /* disable output while we setup */
	fw->CWOutputEnable( false ); /* pulsed mode */

/* setup timing */
	std::vector<int> opCode;
	opCode.push_back( 140 );

	uint32_t delay_ns = 10000;
	uint32_t width_ns = 40000;

	fw->setTrigDelay(  IKlystronFw::LLRF_ACCEL, delay_ns );
	fw->setTrigWidth(  IKlystronFw::LLRF_ACCEL, width_ns );
	fw->setTrigOpCode( IKlystronFw::LLRF_ACCEL, opCode );
	
/* setup IQ table */

	std::vector<double> i_vec(4096, 0);
	std::vector<double> q_vec(4096, 0);

	for ( int i = 0; i < 4096; i++ )
	{
		if ( i < 1071 ) {        /* +1 for first 3 us */
			i_vec[i] = 1.0;
		} else if ( i < 1428 ) {
			i_vec[i] = -1.0; /* -1 for next 1 us */
		} else {
			i_vec[i] = 0;
		}	
	}
	
	fw->setIQSPTable( 4096, (double *) &i_vec[0], (double *) &q_vec[0] );

/* Test SW rotation */
	fw->setSWFeedbackCorrection( IKlystronFw::ONE, 1, 90.0 );

	double start_ns = 0;
	double end_ns   = 4000;
	fw->setApplStart( start_ns );
	fw->setApplEnd( end_ns );

/* Arm DAQ, read streams */
	fw->OutputEnable( true );

	if ( save_stream ) {
		std::vector<uint8_t> buf;
		buf.reserve( 4096*10 );
		fw->setDaqSize( 4096*10 );
		fw->armDaq();
		int i = 0;
		for (std::vector<Stream>::iterator it = strms.begin(); it != strms.end(); ++it, i++ ) {
			std::stringstream fname;
			fname << save_stream << i << ".bin";
			std::ofstream file;
			file.open( fname.str().c_str() );
			uint32_t ret = (*it)->read( (uint8_t *) &buf[0], 10000, 10000 );
			file.write( (char *) &buf[0], ret );
			file.close();
		}
	}	

	if ( dmp_yaml ) {
		fw->dumpConfigToYamlFile( dmp_yaml );
	}


} catch (CPSWError e) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
}
}
