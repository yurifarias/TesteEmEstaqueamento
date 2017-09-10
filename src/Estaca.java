public class Estaca {

    private double posX;
    private double posY;
    private double posZ;
    private double angCravacao;
    private double angProjecao;

    public Estaca(double x, double y, double z, double alfa, double omega) {
        posX = x;
        posY = y;
        posZ = z;
        angCravacao = alfa;
        angProjecao = omega;
    }

    public double getPosX() {
        return posX;
    }

    public double getPosY() {
        return posY;
    }

    public double getPosZ() {
        return posZ;
    }

    public double getAngCrav() {
        return angCravacao;
    }

    public double getAngProj() {
        return angProjecao;
    }
}