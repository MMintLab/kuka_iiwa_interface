/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package armlab.lcm.msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class robotiq_3finger_actuator_command implements lcm.lcm.LCMEncodable
{
    public double timestamp;
    public double position;
    public double speed;
    public double force;
 
    public robotiq_3finger_actuator_command()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x530204c46cbbbb36L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(armlab.lcm.msgs.robotiq_3finger_actuator_command.class))
            return 0L;
 
        classes.add(armlab.lcm.msgs.robotiq_3finger_actuator_command.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        outs.writeDouble(this.timestamp); 
 
        outs.writeDouble(this.position); 
 
        outs.writeDouble(this.speed); 
 
        outs.writeDouble(this.force); 
 
    }
 
    public robotiq_3finger_actuator_command(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public robotiq_3finger_actuator_command(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static armlab.lcm.msgs.robotiq_3finger_actuator_command _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        armlab.lcm.msgs.robotiq_3finger_actuator_command o = new armlab.lcm.msgs.robotiq_3finger_actuator_command();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.timestamp = ins.readDouble();
 
        this.position = ins.readDouble();
 
        this.speed = ins.readDouble();
 
        this.force = ins.readDouble();
 
    }
 
    public armlab.lcm.msgs.robotiq_3finger_actuator_command copy()
    {
        armlab.lcm.msgs.robotiq_3finger_actuator_command outobj = new armlab.lcm.msgs.robotiq_3finger_actuator_command();
        outobj.timestamp = this.timestamp;
 
        outobj.position = this.position;
 
        outobj.speed = this.speed;
 
        outobj.force = this.force;
 
        return outobj;
    }
 
}

