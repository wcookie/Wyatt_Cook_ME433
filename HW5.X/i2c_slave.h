/* 
 * File:   i2c_slave.h
 * Author: Daniel
 *
 * Created on April 17, 2017, 9:25 PM
 */

#ifndef I2C_SLAVE_H
#define	I2C_SLAVE_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* I2C_SLAVE_H */

void __ISR(_I2C_2_VECTOR, IPL1SOFT) I2C2SlaveInterrupt(void);

void i2c_slave_setup(unsigned char addr);


