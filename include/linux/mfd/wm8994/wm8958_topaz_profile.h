#ifndef WM8958_TOPAZ_PROFILE
#define WM8958_TOPAZ_PROFILE


/* Path type */
#define NONE		     0 
#define HEADSET_RX 1
#define SPEAKER_RX  2
#define HANDSET_TX 3
#define HEADSET_TX 4
#define SPEAKER_HEADSET_RX 5

/* default power up and down sequence index*/
#define HEADPHONE_COLD_STARTUP   				0
#define HEADPHONE_WARM_STARTUP  				8
#define SPEAKER_STARTUP 						16
#define EARPIECE_STARTUP 						19
#define LINEOUT_STARTUP 						25
#define SPEAKER_HEADPHONE_SHUTDOWN 			34
#define GENERIC_SHUTDOWN						42


#define SEQ_LINEOUT_HEAD						0
#define SEQ_LINEOUT_ACOUSTIC_HEAD			1
#define SEQ_HEADPHONE_HEAD					2
#define SEQ_HEADPHONE_ACOUSTIC_HEAD			3
#define SEQ_DATA								4

struct wm8958_seq_item {
	int seq_type;
	int index;
	unsigned short sequencer_0;
	unsigned short sequencer_1;
	unsigned short sequencer_2;
	unsigned short sequencer_3;
	
};



#endif
