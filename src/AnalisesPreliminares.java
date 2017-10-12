import Jama.Matrix;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.Arrays;

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

    private double modElasticidade;

    private Matrix matrizRigidezEstacas;
    private Matrix matrizComponentesEstacas;
    private Matrix matrizRigidez;

    private char casoSimetria;

    public AnalisesPreliminares() {

        matrizRigidezEstacas = montarMatrizRigidezEstacas();
        matrizComponentesEstacas = montarMatrizComponentesEstacas();
        matrizRigidez = calcularMatrizRigidez();

        casoSimetria = definirCaso();
    }

    public void calcularCaso(char caso) {

        if (caso == 'A') {

            EstacasVerticais estacasVerticais = new EstacasVerticais();

            MainActivity.reacoesNormais = estacasVerticais.calcularEsforcosNormais();

        } else if (caso == 'B' || caso == 'C') {

            EstaqueamentoPlano estaqueamentoPlano = new EstaqueamentoPlano();

            MainActivity.reacoesNormais = estaqueamentoPlano.calcularEsforcosNormais(caso);

        } else if (caso == 'D' || caso == 'E') {

            SimetriaPorUmPlano simetriaPorUmPlano = new SimetriaPorUmPlano();

            MainActivity.reacoesNormais = simetriaPorUmPlano.calcularEsforcosNormais(caso);

        } else if (caso == 'F') {

            SimetriaPorDoisPlanos simetriaPorDoisPlanos = new SimetriaPorDoisPlanos();

            MainActivity.reacoesNormais = simetriaPorDoisPlanos.calcularEsforcosNormais();

        } else if (caso == 'G') {

            SimetriaPorUmEixo simetriaPorUmEixo = new SimetriaPorUmEixo();

            MainActivity.reacoesNormais = simetriaPorUmEixo.calcularEsforcosNormais();

        } else {

            try {

                Matrix matrizMovElastico = matrizRigidez.solve(new Matrix(MainActivity.esforcos));
                MainActivity.reacoesNormais = (matrizComponentesEstacas.transpose()).times(matrizMovElastico);

            } catch (Exception e) {

                System.out.println("Não é possível achar reações.");
            }
        }

        try {

            System.out.println(Arrays.deepToString(MainActivity.reacoesNormais.getArray()));

        } catch (Exception e) {
        }
    }

    private Matrix montarMatrizRigidezEstacas() {

        determinarFckConcreto();

        double[][] matriz = new double[MainActivity.estaqueamento.length][MainActivity.estaqueamento.length];

        double rigidezEstaca = (modElasticidade * Math.PI * Math.pow(MainActivity.diametroEstacas, 2)) / (4 * MainActivity.comprimentoEstacas);

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            for (int j = 0; j < MainActivity.estaqueamento.length; j++) {

                if (i == j) {

                    matriz[i][j] = rigidezEstaca;

                } else {

                    matriz[i][j] = 0;
                }
            }
        }

        return new Matrix(matriz);
    }

    /* Método para calcular as componentes vetoriais de cada estaca i, retornando
    a matriz P (6 x n) com as respectivas componentes ordenadas pelo índice i, sendo:
    i o índice da estaca;
    n o número de estacas. */
    private Matrix montarMatrizComponentesEstacas() {

        double[][] matrizP = new double[6][MainActivity.estaqueamento.length];

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {
            Estacas e = MainActivity.estaqueamento[i];

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
    private Matrix calcularMatrizRigidez() {

        return (matrizComponentesEstacas.times(matrizRigidezEstacas)).times(matrizComponentesEstacas.transpose());
    }

    private void determinarFckConcreto() {

        switch (MainActivity.fckConcreto) {
            case "20":

                modElasticidade = 21 * Math.pow(10, 9);
                break;

            case "25":

                modElasticidade = 24 * Math.pow(10, 9);
                break;

            case "30":

                modElasticidade = 27 * Math.pow(10, 9);
                break;

            case "35":

                modElasticidade = 29 * Math.pow(10, 9);
                break;

            case "40":

                modElasticidade = 32 * Math.pow(10, 9);
                break;

            case "45":

                modElasticidade = 34 * Math.pow(10, 9);
                break;

            case "50":

                modElasticidade = 37 * Math.pow(10, 9);
                break;
        }
    }

    private char definirCaso() {

        testeQuadrantes();

        double somaYi = 0;
        double somaZi = 0;
        double somaCosAi = 0;
        double somaSenAiCosWi = 0;
        double somaSenAiSenWi = 0;

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {
            Estacas e = MainActivity.estaqueamento[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            somaYi += yi;
            somaZi += zi;
            somaCosAi += cosAi(ai);
            somaSenAiCosWi += senAicosWi(ai, wi);
            somaSenAiSenWi += senAisenWi(ai, wi);

        }

        /* Todas estacas paralelas, ou seja, o ângulo de cravação
         de todas as estacas é de 90 graus com o plano do solo */
        if (somaCosAi == MainActivity.estaqueamento.length) {

            return 'A';

        } /* Estaqueamento plano no plano XY */
        else if (testePlanoXY()) {

            return 'B';

        } /* Estaqueamento plano no plano XZ */
        else if (testePlanoXZ()) {

            return 'C';

        } /* Plano XY de simetria */
        else if (testeSimetriaXY1() && testeSimetriaXY2() && testeSimetriaXY3() && testeSimetriaXY4() &&
                (!testeSimetriaXZ1() || !testeSimetriaXZ2() || !testeSimetriaXZ3() || !testeSimetriaXZ4())) {

            return 'D';

        } /* Plano XZ de simetria */
        else if (testeSimetriaXZ1() && testeSimetriaXZ2() && testeSimetriaXZ3() && testeSimetriaXZ4() &&
                (!testeSimetriaXY1() || !testeSimetriaXY2() || !testeSimetriaXY3() || !testeSimetriaXY4())) {

            return 'E';

        }  /* Simetria por dois planos (XY e XZ) */
        else if (testeSimetriaXY1() && testeSimetriaXY2() && testeSimetriaXY3() && testeSimetriaXY4() &&
                testeSimetriaXZ1() && testeSimetriaXZ2() && testeSimetriaXZ3() && testeSimetriaXZ4()) {

            return 'F';

        } /* Simetria pelo eixo x */
        else if ((!testeSimetriaXY1() || !testeSimetriaXY2() || !testeSimetriaXY3()) && testeSimetriaXY4() &&
                (!testeSimetriaXZ1() || !testeSimetriaXZ2() || !testeSimetriaXZ3()) && testeSimetriaXZ4() &&
                somaYi == 0 && somaZi == 0 && somaSenAiCosWi == 0 && somaSenAiSenWi == 0) {

            return 'G';

        } /* Caso geral */
        else {

            return 'Z';
        }
    }

    private void testeQuadrantes() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    /* Método para testar se todas as estacas estão contidas no plano XY. */
    private boolean testePlanoXY() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if ((eEI.contains(estacai) || eEII.contains(estacai) || eZero.contains(estacai) && wi == 0)
                    || (eEI.contains(estacai) || eEII.contains(estacai) || eZero.contains(estacai) && wi == 180)) {

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

    /* Método para testar se todas as estacas estão contidas no plano XZ. */
    private boolean testePlanoXZ() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

            double yi = e.getPosY();
            double zi = e.getPosZ();
            double ai = e.getAngCrav();
            double wi = e.getAngProj();

            String estacai = yi + ", " + zi + ", " + ai + ", " + wi;

            if ((eEIII.contains(estacai) || eEIV.contains(estacai) || eZero.contains(estacai) && ai == 0)
                    || (eEIII.contains(estacai) || eEIV.contains(estacai) || eZero.contains(estacai) && wi == 90)
                    || (eEIII.contains(estacai) || eEIV.contains(estacai) || eZero.contains(estacai) && wi == 270)) {

                ePlanoXZ.add(estacai);

            } else {

                ePlanoXZ.add("false");
            }
        }

        if (!ePlanoXZ.contains("false")) {

            return true;

        } else {

            return false;
        }
    }

    private boolean testeSimetriaXY1() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXY2() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXY3() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXY4() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXZ1() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXZ2() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXZ3() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private boolean testeSimetriaXZ4() {

        for (int i = 0; i < MainActivity.estaqueamento.length; i++) {

            Estacas e = MainActivity.estaqueamento[i];

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

    private double cosAi(double ai) {

        double cosAi = Math.cos(Math.toRadians(ai));

        if (cosAi == 0) {

            return cosAi;
        } else {

            BigDecimal bd = new BigDecimal(cosAi).setScale(6, RoundingMode.HALF_EVEN);
            return bd.doubleValue();
        }
    }

    private double senAi(double ai) {

        double senAi = Math.sin(Math.toRadians(ai));

        if (senAi == 0) {

            return senAi;
        } else {

            BigDecimal bd = new BigDecimal(senAi).setScale(6, RoundingMode.HALF_EVEN);
            return bd.doubleValue();
        }
    }

    private double cosWi(double wi) {

        double cosWi;

        if (90 < wi && wi <= 180) {
            cosWi = -Math.cos(Math.toRadians(180 - wi));

        } else if (180 < wi && wi <= 270) {
            cosWi = -Math.cos(Math.toRadians(wi - 180));

        } else if (270 < wi && wi <= 360) {
            cosWi = Math.cos(Math.toRadians(360 - wi));

        } else {
            cosWi = Math.cos(Math.toRadians(wi));

        }

        if (cosWi == 0) {

            return cosWi;
        } else {

            BigDecimal bd = new BigDecimal(cosWi).setScale(6, RoundingMode.HALF_EVEN);
            return bd.doubleValue();
        }
    }

    private double senWi(double wi) {

        double senWi;

        if (90 < wi && wi <= 180) {
            senWi = Math.sin(Math.toRadians(180 - wi));

        } else if (180 < wi && wi <= 270) {
            senWi = -Math.sin(Math.toRadians(wi - 180));

        } else if (270 < wi && wi <= 360) {
            senWi = -Math.sin(Math.toRadians(360 - wi));

        } else {
            senWi = Math.sin(Math.toRadians(wi));

        }

        if (senWi == 0) {

            return senWi;
        } else {

            BigDecimal bd = new BigDecimal(senWi).setScale(6, RoundingMode.HALF_EVEN);
            return bd.doubleValue();
        }
    }

    private double senAicosWi (double ai, double wi) {

        double senAicosWi = senAi(ai)*cosWi(wi);

        if (senAicosWi == 0) {

            return senAicosWi;
        } else {

            BigDecimal bd = new BigDecimal(senAicosWi).setScale(6, RoundingMode.HALF_EVEN);
            return bd.doubleValue();
        }
    }

    private double senAisenWi (double ai, double wi) {

        double senAisenWi = senAi(ai)*senWi(wi);

        if (senAisenWi == 0) {

            return senAisenWi;
        } else {

            BigDecimal bd = new BigDecimal(senAisenWi).setScale(6, RoundingMode.HALF_EVEN);
            return bd.doubleValue();
        }
    }

    protected Matrix getMatrizRigidezEstacas() {
        return matrizRigidezEstacas;
    }

    protected Matrix getMatrizComponentesEstacas() {
        return matrizComponentesEstacas;
    }

    protected Matrix getMatrizRigidez() {
        return matrizRigidez;
    }

    protected char getCasoSimetria() {
        return casoSimetria;
    }
}