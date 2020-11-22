//Digital Logic - Cruise Control Project
//Smith, Pham, Nguyen
//12/9/2019

//This code was tested on DE2i FPGA board

/*		Our team designed a cruise control system which allows the driver to activate
and controll the speed of the car without the accelerator or the brake. 
The system also maintains the speed of the car when the cars speed is equal
to the desired speed set by the driver. */




module Cruise_Control (
input clk,
input accelerator,		// Simulates the gas
input brake,			// Simulates brake. Driver can controll car's speed with the accelerator and brake at any point
input cc, 				// Cruise control on/off (system on/off)
input add,				// +1 to desired speed
input subtract,			// -1 from desired speed
input too_close,		// Simulates a sensor on car that sends a HIGH signal when car is less than ten feet behind another car
input approaching_object,		// Simulates a sensor on car that sends a HIGH signal when car is approaching an obstacle too fast
output reg [5:0] ledtest,		// LEDs display the current state (see state transition block)
output reg [1:0] led,			// LEDr3 simulates alerting the driver that they are approaching an object. LEDg0 displays that cruise control is on
output reg [0:6] seg1c,			// Current speed 10's place (current speed on left)
output reg [0:6] seg0c,			// Current speed 1's place
output reg [0:6] seg1d,			// Desired speed 10's place (desired speed on right)
output reg [0:6] seg0d			// Desired speed 1's place
);



// Initializations
reg [6:0] current_speed;
reg [6:0] desired_speed;
reg [3:0] cseg0;
reg [3:0] dseg0;
integer cseg1;
integer dseg1;

localparam OFF = 0, CC = 1, ACCELa = 2, DECEL = 3, ACCELb = 4, ALERT = 5;	// Five states
reg [2:0] current_state;
reg [2:0] next_state;

//new clock
reg [25:0] count = 0;
reg new_clk;
parameter max_count = 50000000/4;	// New clock cycle set to 2 per second

always @(posedge clk) begin
	if (count < max_count) begin
		count <= count + 1;
	end else begin
		count <= 0;
		new_clk <= ~new_clk;
	end
end




// State transition
always @* begin															// Driver is in control of speed
	next_state = current_state;
	case (current_state)							
		
		OFF: begin															// Reset state. Current speed will not reset.
			if (cc == 0) next_state = CC;
		end
		
		CC: begin															// System is in control of speed (driver can immediately take control over the cars speed during any state)
			if (approaching_object == 1 & accelerator == 0) next_state = ALERT;	// Proximity sensor detects car is approaching an object too fast.
			else if (too_close == 1) next_state = DECEL;						// Proximity sensor detects car is too close to object ahead.
			else if (cc == 0) next_state = OFF;									// Cruise control turned off by user.
			else if (brake == 1) next_state = OFF;								// Driver has pressed the brake, turning cruise control off.
			else if (accelerator == 1) next_state = ACCELb;						// Driver has pressed the accelerator, temporarily turning off cruise control.
			else if (current_speed > desired_speed) next_state = DECEL;			// Car's speed is greater than the desired speed.
			else if (current_speed < desired_speed) next_state = ACCELa;		// Car's speed is less than the desirred speed.
		end
		
		ACCELa: begin														// System is in control of speed.
			if (approaching_object == 1) next_state = ALERT;					// Approaching object. Proximity sensor detects car is approaching an object too fast.
			else if (too_close == 1) next_state = DECEL;						// Proximity sensor detects car is approaching an object too fast.
			else if (cc == 0) next_state = OFF;									// Cruise control turned off by user.
			else if (brake == 1) next_state = OFF;								// Driver has pressed the brake, turning cruise control off.
			else if (accelerator == 1) next_state = ACCELb;						// Driver has pressed the accelerator, temporarily turning off cruise control.
			else if (current_speed >= desired_speed) next_state = CC;			// Car's speed is greater than the desired speed.
		end
		
		DECEL: begin														// System is in control of speed.
			if (approaching_object == 1) next_state = ALERT;					// Approaching object. Proximity sensor detects car is approaching an object too fast.
			else if (brake == 1) next_state = OFF;								// Driver has pressed the brake, turning cruise control off.
			else if (cc == 0) next_state = OFF;									// Cruise control turned off by user.
			else if (accelerator == 1) next_state = ACCELb;						// Driver has pressed the accelerator, temporarily turning off cruise control.
			else if (too_close == 0 & current_speed < desired_speed) next_state = CC;	// As long as car is no longer apporaching an object and current speed is less than desired speed, stop decelerating
			else if (current_speed == desired_speed) next_state = CC;			// Car's speed is eqaul to desired speed.
		end
		
		ACCELb: begin														// Driver is in control of speed.
			if (brake == 1) next_state = OFF;									// Driver has pressed the brake, turning cruise control off.
			else if (cc == 0) next_state = OFF;									// Cruise control turned off by user.
			else if (accelerator == 0) next_state = CC;							// Driver has released the accelerator.
		end
		
		ALERT: begin														// System is in control of speed
			if (approaching_object == 0) next_state = CC;						// Car is no longer approaching an object.
			else if (brake == 1) next_state = OFF;								// Driver has pressed the brake, turning cruise control off.
			else if (accelerator == 1) next_state = ACCELb;						// Driver has pressed the accelerator, temporarily turning off cruise control.
		end
endcase
end




//current speed for each state. 
//Adds or subtracts desired speed and current speed according to states and inputs
// Car's hypothetical speed boundaries are 0mph - 64mph
always @(posedge new_clk) begin
		if (current_speed < 64 & add == 0) desired_speed = desired_speed + 1;		// Increment car's desired speed on next clock cycle.
		if (current_speed > 0 & subtract == 0) desired_speed = desired_speed - 1;	// Decrement car's desired speed on next clock cycle.
		if (desired_speed < 1) desired_speed = 0;									// Desired speed is 0. Prevents desired speed from going lowere than 0.
		if (desired_speed > 63) desired_speed = 64;									// Desired speed is 64. Prevents desired speed from going higher than 64.
		if (current_speed < 1) current_speed = 0;									// Current speed is 0. Prevents current speed from going lower than 0.
		if (current_speed > 63) current_speed = 64;									// Current speed is 64. Prevents current speed from going higher than 64.
	current_state = next_state;
	case (current_state)
		OFF: begin	// Driver is in control. Cruise control of off.
			if (current_speed < 64 & accelerator == 1) current_speed = current_speed + 1;	// Increment car's current speed on next clock cycle.
			if (current_speed > 0 & brake == 1) current_speed = current_speed - 1;			// Decrement car's current speed on next clock cycle.
			if (current_speed > 0 & accelerator == 0 & brake == 0) current_speed = current_speed - 1;	// Simulates car's natural deceleration due to friction
			desired_speed = current_speed;
		end
		CC: begin
		end
		ACCELa: begin
			if (current_speed < 64)
				current_speed = current_speed + 1;		// Increment car's current speed on next clock cycle.
		end
		DECEL: begin
			if (current_speed > 0)
			current_speed = current_speed - 1;			// Decrement car's current speed on next clock cycle.
		end
		ACCELb: begin
			if (current_speed < 64 & accelerator == 1)
			current_speed = current_speed + 1;			// Increment car's current speed on next clock cycle.
		end
		ALERT: begin
			if (current_speed > 0)
			current_speed = current_speed - 1;			// Decrement car's current speed on next clock cycle.
		end
	endcase
end




//output block. LED & 7-seg displays
always @* begin
	// 7-seg displays for current speed.
	cseg1 = current_speed / 10;				// Stores tens digit of current speed
	cseg0 = current_speed - cseg1 * 10;		// Stores ones digit of current speed
	case (cseg1)
		0: seg1c = 7'b0000001;
		1: seg1c = 7'b1001111;
		2: seg1c = 7'b0010010;
		3: seg1c = 7'b0000110;
		4: seg1c = 7'b1001100;
		5: seg1c = 7'b0100100;
		6: seg1c = 7'b0100000;
		7: seg1c = 7'b0001111;
		8: seg1c = 7'b0000000;
		9: seg1c = 7'b0000100;
	endcase
	case (cseg0)
		0: seg0c = 7'b0000001;
		1: seg0c = 7'b1001111;
		2: seg0c = 7'b0010010;
		3: seg0c = 7'b0000110;
		4: seg0c = 7'b1001100;
		5: seg0c = 7'b0100100;
		6: seg0c = 7'b0100000;
		7: seg0c = 7'b0001111;
		8: seg0c = 7'b0000000;
		9: seg0c = 7'b0000100;
	endcase
	
	// 7-seg displays for desired speed.
	dseg1 = desired_speed / 10;				// Stores tens digit of desired speed
	dseg0 = desired_speed - dseg1 * 10;		// Stores ones digit of desired speed
	if(current_state == OFF) begin
		seg1d = 7'b1111111;
		seg0d = 7'b1111111;
	end
	else begin
	case (dseg1)
		0: seg1d = 7'b0000001;
		1: seg1d = 7'b1001111;
		2: seg1d = 7'b0010010;
		3: seg1d = 7'b0000110;
		4: seg1d = 7'b1001100;
		5: seg1d = 7'b0100100;
		6: seg1d = 7'b0100000;
		7: seg1d = 7'b0001111;
		8: seg1d = 7'b0000000;
		9: seg1d = 7'b0000100;
	endcase
	case (dseg0)
		0: seg0d = 7'b0000001;
		1: seg0d = 7'b1001111;
		2: seg0d = 7'b0010010;
		3: seg0d = 7'b0000110;
		4: seg0d = 7'b1001100;
		5: seg0d = 7'b0100100;
		6: seg0d = 7'b0100000;
		7: seg0d = 7'b0001111;
		8: seg0d = 7'b0000000;
		9: seg0d = 7'b0000100;
	endcase
	end
	
	// LEDs for current state
	case (current_state)
		OFF: begin 
			led = 2'b00;
			ledtest = 6'b000001;	//State: OFF
		end
		CC: begin
			led = 2'b01;			//CC on
			ledtest = 6'b000010;	//State: CC
		end
		ACCELa: begin
			led = 2'b01;
			ledtest = 6'b000100;	//State: ACCELa
		end
		DECEL: begin
			led = 2'b01;
			ledtest = 6'b001000;	//State: DECEL
		end
		ACCELb: begin
			led = 2'b00;
			ledtest = 6'b010000;	//State: ACCELb
		end
		ALERT: begin
			led = 2'b11;			//Alert on & CC on
			ledtest = 6'b100000;	//State: ALERT
		end
	endcase
end
endmodule