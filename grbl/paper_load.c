#include "grbl.h"


#define PAPER_INIT_FEED 15.0
#define PAPER_EJECT_FEED 50.0



static uint8_t busy;

// Reset Paper Load subsystem.
void pl_reset() {
  busy = false;
}

// Set paper size to zero eg. require paper_load to set proper values
void pg_reset() {
  memset(paper_min_travel, 0, sizeof(paper_min_travel));
  memset(paper_max_travel, 0, sizeof(paper_max_travel));
}

// Paper position cycle
void pl_cycle() {

  if(busy) {
    return;
  }
  if (sys.state == STATE_CHECK_MODE) { 
    return; 
  }
  if(PAPER_IS_OUT && PAPER_Y_IS_OUT) {
    return; 
  }

  busy = true;
  pg_reset();

  uint8_t gc_parser_flags = GC_PARSER_NONE;

  plan_line_data_t pl_data;
  memset(&pl_data, 0, sizeof(plan_line_data_t));
  pl_data.feed_rate = settings.homing_seek_rate;
  pl_data.condition = PL_COND_FLAG_PAPER_LOAD;

  float target[N_AXIS];
  system_convert_array_steps_to_mpos(target, sys_position);
   
  // 
  // 00 - pen up
  //
  target[Z_AXIS] = PEN_UP_POSITION;
  plan_buffer_line(target, &pl_data);
  protocol_buffer_synchronize();

  //
  // 01 - load paper
  //

  target[Y_AXIS] -= settings.max_travel[Y_AXIS];

  // feed paper, ploter is empty
  if(PAPER_Y_IS_OUT) {
    mc_probe_cycle(target, &pl_data, gc_parser_flags, PROBE_Y_BEGIN);
  }
  // there is paper, so this is eject request
  else if(sys.state == STATE_IDLE) {
    // to allow the move
    paper_max_travel[Y_AXIS] = settings.max_travel[Y_AXIS];
    mc_probe_cycle(target, &pl_data, gc_parser_flags, PROBE_Y_END);
    system_convert_array_steps_to_mpos(target, sys_position);
    target[Y_AXIS] += PAPER_EJECT_FEED;
    plan_buffer_line(target, &pl_data);
    pl_reset();
    pg_reset();
    return;
  }
  else {
    // just ignore when doing something
    busy = false;
    return;
  }

  //
  // 02 - detect bottom edge of the paper
  // 

  // move paper over the sensor end set Y=0 position
  if(sys.probe_succeeded) {
    system_convert_array_steps_to_mpos(target, sys_position);
    target[Y_AXIS] += PAPER_INIT_FEED;
    plan_buffer_line(target, &pl_data);
    protocol_buffer_synchronize();
    // this is paper top position
    sys_position[Y_AXIS] = lround(settings.max_travel[Y_AXIS] * settings.steps_per_mm[Y_AXIS]);
    gc_state.coord_offset[Y_AXIS] = settings.max_travel[Y_AXIS];
    paper_max_travel[Y_AXIS] = system_convert_axis_steps_to_mpos(sys_position, Y_AXIS);
    plan_sync_position();
  }
  else {
    // just return, error is reported by mc_probe_cycle()
    busy = false;
    return;
  }

  //
  // 03 - detect right edge of the paper
  //

  // move caret right
  system_convert_array_steps_to_mpos(target, sys_position);
  target[X_AXIS] = -settings.homing_pulloff + settings.tool_x_offset[sys_tool];
  plan_buffer_line(target, &pl_data);
  protocol_buffer_synchronize();

  // check the right edge of the paper
  system_convert_array_steps_to_mpos(target, sys_position);
  target[X_AXIS] = settings.max_travel[X_AXIS];
  mc_probe_cycle(target, &pl_data, gc_parser_flags, PROBE_X_BEGIN);

  if(sys.probe_succeeded) {
    paper_min_travel[X_AXIS] = system_convert_axis_steps_to_mpos(sys_probe_position, X_AXIS) 
      - settings.tool_x_offset[sys_tool];
  }
  else {
    paper_min_travel[X_AXIS] = -settings.homing_pulloff;
    system_clear_exec_alarm(EXEC_ALARM_PROBE_FAIL_INITIAL);
  }

  //
  // 04 - detect left edge of the paper
  //

  // check the left end of the paper
  system_convert_array_steps_to_mpos(target, sys_position);
  target[X_AXIS] = settings.max_travel[X_AXIS];
  mc_probe_cycle(target, &pl_data, gc_parser_flags, PROBE_X_END);

  if(sys.probe_succeeded) {
    float maxTravel = system_convert_axis_steps_to_mpos(sys_probe_position, X_AXIS) 
      - settings.tool_x_offset[sys_tool];
    gc_state.coord_offset[X_AXIS] = maxTravel;
    paper_max_travel[X_AXIS] = maxTravel;
  }
  else {
    // just return, error is reported by mc_probe_cycle()
    busy = false;
    return;
  }

  //
  // 05 - detect top edge of the paper
  //

  // check size of the paper
  system_convert_array_steps_to_mpos(target,sys_position);
  target[Y_AXIS] -= settings.max_travel[Y_AXIS];
  mc_probe_cycle(target, &pl_data, gc_parser_flags, PROBE_Y_END);

  if(sys.probe_succeeded) {
    paper_min_travel[Y_AXIS] = system_convert_axis_steps_to_mpos(sys_probe_position, Y_AXIS);
  }
  else {
    // just return, error is reported by mc_probe_cycle()
    busy = false;
    return;
  }

  system_convert_array_steps_to_mpos(target, sys_position);
  target[Y_AXIS] = settings.max_travel[Y_AXIS];
  target[X_AXIS] -= settings.tool_x_offset[sys_tool];
  gc_state.coord_offset[Z_AXIS] = 0.0;
  plan_buffer_line(target, &pl_data);

  protocol_buffer_synchronize();
  gc_sync_position();
  plan_sync_position();
  system_flag_wco_change();

  busy = false;

}
