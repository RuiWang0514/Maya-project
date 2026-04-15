#ifndef REMOTECOMMAND_H_
#define REMOTECOMMAND_H_

#define FIRSTBYTEA rxbufA+1+8
#define FIRSTBYTEB rxbufB+1+8


void Command_InitA(void);
void Command_InitB(void);

#endif /* REMOTECOMMAND_H_ */
