#pragma once

class MLhelper_base {
  public:
    MLhelper_base() {}
    virtual int begin(long frequency);
	virtual void end();
    virtual int lqi() { return 0; }
	virtual void sleep();
	virtual void idle();
    virtual void setTxPower(int level) {};
	virtual int read(byte* buf, uint8_t len);
	virtual int write(byte* buf, uint8_t len);
	virtual int receiveMode();
	virtual int transmitMode();
    virtual void handleDintRise() {};
	virtual void onInternalRxDone(void(*callback)(int)) {};
	virtual void onInternalTxDone(void(*callback)()) {};
	virtual void setDint(uint8_t pin);
	virtual void setRst(uint8_t pin);

  protected:
  	static void onDintRise();
    void (*_onInternalRxDone)(int);
    void (*_onInternalTxDone)();
    uint8_t _dint;
};
