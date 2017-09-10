import Jama.Matrix;

public class SimetriaPorUmEixo extends AnalisesPreliminares {

    private double fi;
    private double xoYBL;
    private double xoZBL;
    private double xoYCL;
    private double xoZCL;

    public Matrix reacoesNormais(Estaca[] estacas, double[][] esforcosExternos) {

        novasCoordenadas(estacas);

        return mMovElasticoTransformado(estacas, esforcosExternos).transpose().times(mComponentesDeVetorTransformada(estacas));
    }

    /* Método para achar as coordenadas dos novos eixos coordenados (xo, yo), em relação ao antigo,
    e o ângulo de rotação fi, após a translação e rotação dos antigos eixos coordenados como
    aritifício para solucionar casos de estaqueamento simétrico por um plano. */
    private void novasCoordenadas(Estaca[] estacas) {

        double[][] mS = mRigidez(estacas).getArray();

        fi = (Math.atan((2*mS[1][2])/(mS[1][1]-mS[2][2])))/2;

        double[][] mRigidezLinhaB = {{(-mS[1][1] - mS[1][2]*Math.tan(fi)),(mS[2][2] + mS[1][2]/Math.tan(fi))},
                {(mS[1][1]*Math.tan(fi) - mS[1][2]),(mS[2][2]/Math.tan(fi) - mS[1][2])}};

        double[][] mRigidezIgualdadeB = {{-(mS[1][5] + mS[2][4] + mS[1][4]/Math.tan(fi) + mS[2][5]*Math.tan(fi))},
                {-(mS[2][5] - mS[1][4] + mS[2][4]/Math.tan(fi) - mS[1][5]*Math.tan(fi))}};

        double[][] mRigidezSolucaoB = (new Matrix(mRigidezLinhaB)).solve(new Matrix(mRigidezIgualdadeB)).getArray();

        xoYBL = mRigidezSolucaoB[0][0];
        xoZBL = mRigidezSolucaoB[1][0];

        double[][] mRigidezLinhaC = {{(-mS[1][1]/Math.tan(fi) - mS[1][2]),(-mS[2][2]*Math.tan(fi) - mS[1][2])},
                {(mS[1][1] - mS[1][2]/Math.tan(fi)),(-mS[2][2] + mS[1][2]*Math.tan(fi))}};

        double[][] mRigidezIgualdadeC = {{-(mS[2][5] - mS[1][4] + mS[1][5]/Math.tan(fi) - mS[2][4]*Math.tan(fi))},
                {-(mS[1][4]*Math.tan(fi) + mS[2][5]/Math.tan(fi) - mS[1][5] - mS[2][4])}};

        double[][] mSSC = (new Matrix(mRigidezLinhaC)).solve(new Matrix(mRigidezIgualdadeC)).getArray();

        xoYCL = mSSC[0][0];
        xoZCL = mSSC[1][0];

    }

    private Matrix tensorTransformacao() {

        double[][] tensorT = new double[6][6];

        tensorT[0][0] = 1;
        tensorT[0][1] = 0;
        tensorT[0][2] = 0;
        tensorT[0][3] = 0;
        tensorT[0][4] = 0;
        tensorT[0][5] = 0;
        tensorT[1][0] = 0;
        tensorT[1][1] = Math.cos(fi);
        tensorT[1][2] = Math.sin(fi);
        tensorT[1][3] = 0;
        tensorT[1][4] = 0;
        tensorT[1][5] = 0;
        tensorT[2][0] = 0;
        tensorT[2][1] = -Math.sin(fi);
        tensorT[2][2] = Math.cos(fi);
        tensorT[2][3] = 0;
        tensorT[2][4] = 0;
        tensorT[2][5] = 0;
        tensorT[3][0] = 0;
        tensorT[3][1] = 0;
        tensorT[3][2] = 0;
        tensorT[3][3] = tensorT[0][0];
        tensorT[3][4] = tensorT[0][1];
        tensorT[3][5] = tensorT[0][2];
        tensorT[4][0] = 0;
        tensorT[4][1] = xoYBL*Math.sin(fi);
        tensorT[4][2] = -xoZBL*Math.cos(fi);
        tensorT[4][3] = tensorT[1][0];
        tensorT[4][4] = tensorT[1][1];
        tensorT[4][5] = tensorT[1][2];
        tensorT[5][0] = 0;
        tensorT[5][1] = xoYCL*Math.cos(fi);
        tensorT[5][2] = xoZCL*Math.sin(fi);
        tensorT[5][3] = tensorT[2][0];
        tensorT[5][4] = tensorT[2][1];
        tensorT[5][5] = tensorT[2][2];

        return new Matrix(tensorT);
    }

    private Matrix montarMatrizReduzida(Estaca[] estacas) {

        double[][] mSLreduzida = new double[4][4];

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                mSLreduzida[i][j] = (tensorTransformacao().times(mRigidez(estacas))).times(tensorTransformacao().transpose()).getArray()[i][j];
            }
        } return new Matrix(mSLreduzida);
    }

    private Matrix esforcosTransformados(double[][] esforcosExternos) {

        Matrix esforcos = new Matrix(esforcosExternos);

        Matrix esforcosL = tensorTransformacao().times(esforcos);

        double[][] eLreduzidos = new double[4][1];

        for (int i = 0; i < 4; i++) {
            eLreduzidos[i][0] = esforcosL.getArray()[i][0];

        } return new Matrix(eLreduzidos);
    }

    private Matrix mMovElasticoTransformado(Estaca[] estacas, double[][] esforcosExternos) {

        return montarMatrizReduzida(estacas).solve(esforcosTransformados(esforcosExternos));

    }

    private Matrix mComponentesDeVetorTransformada(Estaca[] estacas) {

        Matrix pTransformada = tensorTransformacao().times(mComponentesDeVetor(estacas));

        double[][] pTransformadaReduzida = new double[4][estacas.length];

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < estacas.length; j++) {

                pTransformadaReduzida[i][j] = pTransformada.getArray()[i][j];
            }

        } return new Matrix(pTransformadaReduzida);

    }

    public double getFi() {
        return fi;
    }

    public double getXoYBL() {
        return xoYBL;
    }

    public double getXoZBL() {
        return xoZBL;
    }

    public double getXoYCL() {
        return xoYCL;
    }

    public double getXoZCL() {
        return xoZCL;
    }

}
