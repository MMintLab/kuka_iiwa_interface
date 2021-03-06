/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package armlab.lcm.msgs;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class cartesian_impedance_parameters implements lcm.lcm.LCMEncodable
{
    public armlab.lcm.msgs.cartesian_value_quantity cartesian_stiffness;
    public double nullspace_stiffness;
    public armlab.lcm.msgs.cartesian_value_quantity cartesian_damping;
    public double nullspace_damping;
 
    public cartesian_impedance_parameters()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x46a21e6d7f654ba8L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(armlab.lcm.msgs.cartesian_impedance_parameters.class))
            return 0L;
 
        classes.add(armlab.lcm.msgs.cartesian_impedance_parameters.class);
        long hash = LCM_FINGERPRINT_BASE
             + armlab.lcm.msgs.cartesian_value_quantity._hashRecursive(classes)
             + armlab.lcm.msgs.cartesian_value_quantity._hashRecursive(classes)
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
        this.cartesian_stiffness._encodeRecursive(outs); 
 
        outs.writeDouble(this.nullspace_stiffness); 
 
        this.cartesian_damping._encodeRecursive(outs); 
 
        outs.writeDouble(this.nullspace_damping); 
 
    }
 
    public cartesian_impedance_parameters(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public cartesian_impedance_parameters(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static armlab.lcm.msgs.cartesian_impedance_parameters _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        armlab.lcm.msgs.cartesian_impedance_parameters o = new armlab.lcm.msgs.cartesian_impedance_parameters();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.cartesian_stiffness = armlab.lcm.msgs.cartesian_value_quantity._decodeRecursiveFactory(ins);
 
        this.nullspace_stiffness = ins.readDouble();
 
        this.cartesian_damping = armlab.lcm.msgs.cartesian_value_quantity._decodeRecursiveFactory(ins);
 
        this.nullspace_damping = ins.readDouble();
 
    }
 
    public armlab.lcm.msgs.cartesian_impedance_parameters copy()
    {
        armlab.lcm.msgs.cartesian_impedance_parameters outobj = new armlab.lcm.msgs.cartesian_impedance_parameters();
        outobj.cartesian_stiffness = this.cartesian_stiffness.copy();
 
        outobj.nullspace_stiffness = this.nullspace_stiffness;
 
        outobj.cartesian_damping = this.cartesian_damping.copy();
 
        outobj.nullspace_damping = this.nullspace_damping;
 
        return outobj;
    }
 
}

