struct bluesleepmethods {
  int (*host_wake)(void);
  int (*ext_wake)(void);
  int (*host_wake_irq)(void);
  void (*set_io)(int io, int io_status);
  int (*get_io)(int io);
  int wake;
};