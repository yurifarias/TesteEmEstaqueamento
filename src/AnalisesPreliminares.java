import Jama.Matrix;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;

public class AnalisesPreliminares {

    private ArrayList<String> eQI = new ArrayList<>();
    private ArrayList<String> eQII = new ArrayList<>();
    private ArrayList<String> eQIII = new ArrayList<>();
    private ArrayList<String> eQIV = new ArrayList<>();
    private ArrayList<String> eEI = new ArrayList<>();
    private ArrayList<String> eEII = new ArrayList<>();
    private ArrayList<String> eEIII = new ArrayList<>();
    private ArrayList<String> eEIV = new ArrayList<>();
    private ArrayList<String> eZero = new ArrayList<>();
    private ArrayList<String> ePlanoXY = new ArrayList<>();
    private ArrayList<String> ePlanoXZ = new ArrayList<>();
    private String estacaSimetricaQI;
    private String estacaSimetricaQII;
    private String estacaSimetricaQIII;
    private String estacaSimetricaQIV;
    private String estacaSimetricaEI;
    private String estacaSimetricaEII;
    private String estacaSimetricaEIII;
    private String estacaSimetricaEIV;

    public char tirarSimetria(Estaca[] estaca) {

        testeQuadrantes(estaca);

        double somaYi = 0;
        double somaZi = 0;
        double somaSenAiCosWi = 0;
        double somaSenAiSenWi = 0;
        double somaCosAi = 0;

        for (int i = 0; i < estaca.length; i++) {
            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            somaYi += yi;
            somaZi += zi;
            somaSenAiCosWi += senAicosWi(ai, wi);
            somaSenAiSenWi += senAisenWi(ai, wi);
            somaCosAi += cosAi(ai);

        }

        /* Todas estacas paralelas, ou seja, o ângulo de cravação
         de todas as estacas é de 90 graus com o plano do solo */
        if (somaCosAi == estaca.length) {

            return 'A';

        } /* Estaqueamento plano no plano XY */
        else if (testePlanoXY(estaca) && !testeSimetriaXY1(estaca) && !testeSimetriaXY2(estaca)) {

            return 'B';

        } /* Estaqueamento plano no plano XZ */
        else if (testePlanoXZ(estaca) && !testeSimetriaXZ1(estaca) && !testeSimetriaXZ2(estaca)) {

            return 'C';

        } /* Plano XY de simetria */
        else if (testeSimetriaXY1(estaca) && testeSimetriaXY2(estaca) && testeSimetriaXY3(estaca) && testeSimetriaXY4(estaca) &&
                (!testeSimetriaXZ1(estaca) || !testeSimetriaXZ2(estaca) || !testeSimetriaXZ3(estaca) || !testeSimetriaXZ4(estaca))) {

            return 'D';

        } /* Plano XZ de simetria */
        else if (testeSimetriaXZ1(estaca) && testeSimetriaXZ2(estaca) && testeSimetriaXZ3(estaca) && testeSimetriaXZ4(estaca) &&
                (!testeSimetriaXY1(estaca) || !testeSimetriaXY2(estaca) || !testeSimetriaXY3(estaca) || !testeSimetriaXY4(estaca))) {

            return 'E';

        }  /* Simetria por dois planos (XY e XZ) */
        else if (testeSimetriaXY1(estaca) && testeSimetriaXY2(estaca) && testeSimetriaXY3(estaca) && testeSimetriaXY4(estaca) &&
                testeSimetriaXZ1(estaca) && testeSimetriaXZ2(estaca) && testeSimetriaXZ3(estaca) && testeSimetriaXZ4(estaca)) {

            return 'F';

        } /* Simetria pelo eixo x */
        else if ((!testeSimetriaXY1(estaca) || !testeSimetriaXY2(estaca) || !testeSimetriaXY3(estaca)) && testeSimetriaXY4(estaca) &&
                (!testeSimetriaXZ1(estaca) || !testeSimetriaXZ2(estaca) || !testeSimetriaXZ3(estaca)) && testeSimetriaXZ4(estaca) &&
                somaYi == 0 && somaZi == 0 && somaSenAiCosWi == 0 && somaSenAiSenWi == 0) {

            return 'G';

        } /* Caso geral */
        else {

            return 'Z';

        }
    }


    /* Método para calcular as componentes vetoriais de cada estaca i, retornando
        a matriz P (6 x n) com as respectivas componentes ordenadas pelo índice i, sendo:
        i o índice da estaca;
        n o número de estacas. */
    protected Matrix mComponentesDeVetor(Estaca[] estacas) {

        double[][] matrizP = new double[6][estacas.length];

        for (int i = 0; i < estacas.length; i++) {
            Estaca e = estacas[i];

            double xi = e.getPosX();
            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            double pxi = cosAi(ai);
            double pyi = senAi(ai)*cosWi(wi);
            double pzi = senAi(ai)*senWi(wi);
            double pai = yi * pzi - zi * pyi;
            double pbi = zi * pxi - xi * pzi;
            double pci = xi * pyi - yi * pxi;

            matrizP[0][i] = pxi;
            matrizP[1][i] = pyi;
            matrizP[2][i] = pzi;
            matrizP[3][i] = pai;
            matrizP[4][i] = pbi;
            matrizP[5][i] = pci;
        }
        return new Matrix(matrizP);
    }

    /* Método para a montagem da matriz de rigidez geral S pela multiplicação
    da matriz P por sua transposta, retornando a matriz S (6 x 6). */
    protected Matrix mRigidez(Estaca[] estacas) {

        return mComponentesDeVetor(estacas).times(mComponentesDeVetor(estacas).transpose());
    }

    public char getPlanoSimetria(Estaca[] estacas) {
        return tirarSimetria(estacas);
    }

    private void testeQuadrantes(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            if (yi > 0 && zi > 0) {

				/* A estaca i esta no 1o Quadrante (QI) */
                eQI.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi < 0 && zi > 0) {

				/* A estaca i esta no 2o Quadrante (QII) */
                eQII.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi < 0 && zi < 0) {

				/* A estaca i esta no 3o Quadrante (QIII) */
                eQIII.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi > 0 && zi < 0) {

				/* A estaca i esta no 4o Quadrante (QIV) */
                eQIV.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi > 0 && zi == 0) {

				/* A estaca i esta no segmento positivo do eixo Y (EI) */
                eEI.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi < 0 && zi == 0) {

				/* A estaca i esta no segmento negativo do eixo Y (EII) */
                eEII.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi == 0 && zi > 0) {

				/* A estaca i esta no segmento positivo do eixo Z (EIII) */
                eEIII.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi == 0 && zi < 0) {

				/* A estaca i esta no segmento negativo do eixo Z (EIV) */
                eEIV.add(yi + ", " + zi + ", " + ai + ", " + wi);

            } else if (yi == 0 && zi == 0) {

				/* A estaca i esta na origem do sistema de eixos coordenados */
                eZero.add(yi + ", " + zi + ", " + ai + ", " + wi);

            }
        }
    }

    protected boolean testeSimetriaXY1(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eQI.contains(estacai)) {
                estacaSimetricaQIV = yi + ", " + (-zi) + ", " + ai + ", " + (360 - wi);

            } else if (eQIV.contains(estacai)) {
                estacaSimetricaQI = yi + ", " + (-zi) + ", " + ai + ", " + (360 - wi);

            }
        }

        if (eQI.size() == eQIV.size() && eQI.contains(estacaSimetricaQI) && eQIV.contains(estacaSimetricaQIV)) {

            return true;

        } else {

            return false;

        }
    }

    protected boolean testeSimetriaXY2(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eQII.contains(estacai)) {
                estacaSimetricaQIII = yi + ", " + (-zi) + ", " + ai + ", " + (360 - wi);

            } else if (eQIII.contains(estacai)) {
                estacaSimetricaQII = yi + ", " + (-zi) + ", " + ai + ", " + (360 - wi);

            }
        }

        if (eQII.size() == eQIII.size() && eQII.contains(estacaSimetricaQII) && eQIII.contains(estacaSimetricaQIII)) {
            return true;

        } else {
            return false;

        }
    }

    protected boolean testeSimetriaXY3(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eEIII.contains(estacai)) {
                estacaSimetricaEIV = yi + ", " + (-zi) + ", " + ai + ", " + (360 - wi);

            } else if (eEIV.contains(estacai)) {
                estacaSimetricaEIII = yi + ", " + (-zi) + ", " + ai + ", " + (360 - wi);

            } else if (eEI.contains(estacai) && (ai == 0 || (ai != 0 && wi == 0))) {
                estacaSimetricaEI = estacai;

            } else if (eEII.contains(estacai) && (ai == 0 || ai != 0 && wi == 180)) {
                estacaSimetricaEII = estacai;

            }
        }

        if (eEIII.size() == eEIV.size() && eEIII.contains(estacaSimetricaEIII) && eEIV.contains(estacaSimetricaEIV)) {
            return true;

        } else if ((eEI.contains(estacaSimetricaEI) && !eEI.contains(null)) || (eEI.contains(null) && !eEI.contains(estacaSimetricaEII))
                || (eEI.contains(estacaSimetricaEI) && !eEI.contains(estacaSimetricaEII))) {
            return true;

        } else {
            return false;

        }
    }

    protected boolean testeSimetriaXY4(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (!eZero.contains(estacai)) {
                return true;

            } else if (eZero.contains(estacai) && ai == 0) {
                return true;

            } else if (eZero.contains(estacai) && ai != 0 && (wi == 0 || wi == 180)) {
                return true;

            }

        } return false;
    }

    protected boolean testeSimetriaXZ1(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eQI.contains(estacai)) {
                estacaSimetricaQII = (-yi) + ", " + zi + ", " + ai + ", " + (180 - wi);

            } else if (eQII.contains(estacai)) {
                estacaSimetricaQI = (-yi) + ", " + zi + ", " + ai + ", " + (180 - wi);

            }
        }

        if (eQI.size() == eQII.size() && eQI.contains(estacaSimetricaQI) && eQII.contains(estacaSimetricaQII)) {

            return true;

        } else {

            return false;

        }
    }

    protected boolean testeSimetriaXZ2(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eQIII.contains(estacai)) {
                estacaSimetricaQIV = (-yi) + ", " + zi + ", " + ai + ", " + (540 - wi);

            } else if (eQIV.contains(estacai)) {
                estacaSimetricaQIII = (-yi) + ", " + zi + ", " + ai + ", " + (540 - wi);

            }
        }

        if (eQIII.size() == eQIV.size() && eQIII.contains(estacaSimetricaQIII) && eQIV.contains(estacaSimetricaQIV)) {
            return true;

        } else {
            return false;

        }
    }

    protected boolean testeSimetriaXZ3(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eEI.contains(estacai)) {
                estacaSimetricaEII = (-yi) + ", " + zi + ", " + ai + ", " + (180 - wi);

            } else if (eEII.contains(estacai)) {
                estacaSimetricaEI = (-yi) + ", " + zi + ", " + ai + ", " + (180 - wi);

            } else if (eEIII.contains(estacai) && (ai == 0 || (ai != 0 && wi == 90))) {
                estacaSimetricaEIII = estacai;

            } else if (eEIV.contains(estacai) && (ai == 0 || ai != 0 && wi == 270)) {
                estacaSimetricaEIV = estacai;

            }
        }

        if (eEI.size() == eEII.size() && eEI.contains(estacaSimetricaEI) && eEII.contains(estacaSimetricaEII)) {
            return true;

        } else if ((eEIII.contains(estacaSimetricaEIII) && !eEIV.contains(null)) || (eEIII.contains(null) && !eEIV.contains(estacaSimetricaEIV))
                || (eEIII.contains(estacaSimetricaEIII) && !eEIV.contains(estacaSimetricaEIV))) {
            return true;

        } else {
            return false;

        }
    }

    protected boolean testeSimetriaXZ4(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (!eZero.contains(estacai)) {
                return true;

            } else if (eZero.contains(estacai) && ai == 0) {
                return true;

            } else if (eZero.contains(estacai) && ai != 0 && (wi == 90 || wi == 270)) {
                return true;

            }

        } return false;
    }

    protected boolean testePlanoXY(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eEI.contains(estacai) || eEII.contains(estacai) || eZero.contains(estacai)) {

                ePlanoXY.add("true");

            } else {

                ePlanoXY.add("false");

            }
        }

        if (!ePlanoXY.contains("false")) {

            return true;

        } else {

            return false;

        }
    }

    protected boolean testePlanoXZ(Estaca[] estaca) {

        for (int i = 0; i < estaca.length; i++) {

            Estaca e = estaca[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if (eEIII.contains(estacai) || eEIV.contains(estacai) || eZero.contains(estacai)) {

                ePlanoXZ.add("true");

            } else {

                ePlanoXZ.add("false");

            }
        }

        if (!ePlanoXZ.contains("false")) {

            return true;

        } else {

            return true;

        }
    }

    private double cosAi(double ai) {

        double cosAi = Math.cos(Math.toRadians(ai));

        BigDecimal bd = new BigDecimal(cosAi).setScale(12, RoundingMode.HALF_EVEN);
        return bd.doubleValue();
    }

    private double senAi(double ai) {

        double senAi = Math.sin(Math.toRadians(ai));

        BigDecimal bd = new BigDecimal(senAi).setScale(12, RoundingMode.HALF_EVEN);
        return bd.doubleValue();
    }

    private double cosWi(double wi) {

        double cosWi;

        if (90 < wi && wi <= 180) {
            cosWi = -Math.cos(Math.toRadians(180 - wi));

        }
        if (180 < wi && wi <= 270) {
            cosWi = -Math.cos(Math.toRadians(wi - 180));

        }
        if (270 < wi && wi <= 360) {
            cosWi = Math.cos(Math.toRadians(360 - wi));

        } else {
            cosWi = Math.cos(Math.toRadians(wi));

        }

        BigDecimal bd = new BigDecimal(cosWi).setScale(12, RoundingMode.HALF_EVEN);
        return bd.doubleValue();
    }

    private double senWi(double wi) {

        double senWi;

        if (90 < wi && wi <= 180) {
            senWi = Math.sin(Math.toRadians(180 - wi));

        }
        if (180 < wi && wi <= 270) {
            senWi = -Math.sin(Math.toRadians(wi - 180));

        }
        if (270 < wi && wi <= 360) {
            senWi = -Math.sin(Math.toRadians(360 - wi));

        } else {
            senWi = Math.sin(Math.toRadians(wi));

        }

        BigDecimal bd = new BigDecimal(senWi).setScale(12, RoundingMode.HALF_EVEN);
        return bd.doubleValue();
    }

    private double senAicosWi (double ai, double wi) {

        double senAi = Math.sin(Math.toRadians(ai));
        double cosWi;

        if (90 < wi && wi <= 180) {
            cosWi = -Math.cos(Math.toRadians(180 - wi));

        }
        if (180 < wi && wi <= 270) {
            cosWi = -Math.cos(Math.toRadians(wi - 180));

        }
        if (270 < wi && wi <= 360) {
            cosWi = Math.cos(Math.toRadians(360 - wi));

        } else {
            cosWi = Math.cos(Math.toRadians(wi));

        }

        double senAicosWi = senAi*cosWi;

        BigDecimal bd = new BigDecimal(senAicosWi).setScale(12, RoundingMode.HALF_EVEN);
        return bd.doubleValue();

    }

    private double senAisenWi (double ai, double wi) {

        double senAi = Math.sin(Math.toRadians(ai));
        double senWi;

        if (90 < wi && wi <= 180) {
            senWi = -Math.cos(Math.toRadians(180 - wi));

        }
        if (180 < wi && wi <= 270) {
            senWi = -Math.cos(Math.toRadians(wi - 180));

        }
        if (270 < wi && wi <= 360) {
            senWi = Math.cos(Math.toRadians(360 - wi));

        } else {
            senWi = Math.cos(Math.toRadians(wi));

        }

        double senAisenWi = senAi*senWi;

        BigDecimal bd = new BigDecimal(senAisenWi).setScale(12, RoundingMode.HALF_EVEN);
        return bd.doubleValue();

    }

}