		if ((int)(data8[0]=serialGetchar(fd)) == -1){
			fprintf(stderr, "serialGetchar, block for 10s: %s\n", strerror (errno));
			return 1;
		}
		else  if (data8[0] == 0xff){
		data8[1] = serialGetchar(fd);	// Data type: 0x47
		data8[2] = serialGetchar(fd);
		data8[3] = serialGetchar(fd);
		data8[4] = serialGetchar(fd);
		n = (int)(data8[4]);
		//printf("%d", n);
		for (i=0;i<n;i++){
			data8[5+i] = serialGetchar(fd);
		};
		data8[5+n] = serialGetchar(fd);
		data8[6+n] = serialGetchar(fd);		
		
		if ((data16[0]=data8[3] << 8| data8[2]) != 0x0011){
			printf("Unrecognized code of data in packet -> %04x\n", data16[0]);
		};
		
		pos_raw[0] = (float)((data32[1] = data8[12] << 24| data8[11] << 16| data8[10] << 8| data8[9]));
		pos_raw[1] = (float)((data32[2] = data8[16] << 24| data8[15] << 16| data8[14] << 8| data8[13]));	
		pos_raw[2] = (float)((data32[3] = data8[20] << 24| data8[19] << 16| data8[18] << 8| data8[17]));
		
		printf("X=%.3f, Y=%.3f, Z=%.3f\n\n", pos_raw[0]/1000, pos_raw[1]/1000, pos_raw[2]/1000);
