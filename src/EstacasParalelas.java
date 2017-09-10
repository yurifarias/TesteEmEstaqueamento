import Jama.Matrix;

public class EstacasParalelas extends AnalisesPreliminares{

    private double fi;
    private double yoL;
    private double zoL;

    public Matrix reacoesNormais(Estaca[] estacas, double[][] esforcosExternos) {

        novasCoordenadas(estacas);

        return mMovElasticoTransformado(estacas, esforcosExternos).transpose().times(mComponentesDeVetorTransformada(estacas));
    }

    /* Método para achar as coordenadas dos novos eixos coordenados (xo, yo), em relação ao antigo,
    e o ângulo de rotação fi, após a translação e rotação dos antigos eixos coordenados como
    aritifício para solucionar casos de estaqueamento simétrico por um plano. */
    private void novasCoordenadas(Estaca[] estacas) {

        double[][] mS = mRigidez(estacas).getArray();

        if (testeSimetriaXY1(estacas) && testeSimetriaXY2(estacas) && testeSimetriaXY3(estacas) && testeSimetriaXY4(estacas) &&
                testeSimetriaXZ1(estacas) && testeSimetriaXZ2(estacas) && testeSimetriaXZ3(estacas) && testeSimetriaXZ4(estacas)) {

            fi = 0;

        } else {

            fi = (Math.atan((2 * (mS[0][0] * mS[4][5] - mS[0][4] * mS[0][5]) / (mS[0][0] * (mS[5][5] - mS[4][4]) + Math.pow(mS[0][4], 2) - Math.pow(mS[0][5], 2))))) / 2;
        }

        yoL = (mS[0][5]*Math.cos(fi) + mS[0][4]*Math.sin(fi))/mS[0][0];
        zoL = (mS[0][5]*Math.sin(fi) - mS[0][4]*Math.cos(fi))/mS[0][0];

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
        tensorT[1][2] = -Math.sin(fi);
        tensorT[1][3] = 0;
        tensorT[1][4] = 0;
        tensorT[1][5] = 0;
        tensorT[2][0] = 0;
        tensorT[2][1] = Math.sin(fi);
        tensorT[2][2] = Math.cos(fi);
        tensorT[2][3] = 0;
        tensorT[2][4] = 0;
        tensorT[2][5] = 0;
        tensorT[3][0] = 0;
        tensorT[3][1] = yoL*Math.sin(fi) - zoL*Math.cos(fi);
        tensorT[3][2] = yoL*Math.cos(fi) - zoL*Math.sin(fi);
        tensorT[3][3] = tensorT[0][0];
        tensorT[3][4] = tensorT[0][1];
        tensorT[3][5] = tensorT[0][2];
        tensorT[4][0] = zoL;
        tensorT[4][1] = 0;
        tensorT[4][2] = 0;
        tensorT[4][3] = tensorT[1][0];
        tensorT[4][4] = tensorT[1][1];
        tensorT[4][5] = tensorT[1][2];
        tensorT[5][0] = -yoL;
        tensorT[5][1] = 0;
        tensorT[5][2] = 0;
        tensorT[5][3] = tensorT[2][0];
        tensorT[5][4] = tensorT[2][1];
        tensorT[5][5] = tensorT[2][2];

        return new Matrix(tensorT);
    }

    private Matrix mRigidezTransformada(Estaca[] estacas) {

        return (tensorTransformacao().times(mRigidez(estacas))).times(tensorTransformacao().transpose());

    }

    private Matrix esfExternosTransformados(double[][] esfExternos) {

        return tensorTransformacao().times(new Matrix(esfExternos));
    }

    private Matrix mMovElasticoTransformado(Estaca[] estacas, double[][] esfExternos) {

        double[][] mVL = new double[6][1];

        mVL[0][0] = esfExternosTransformados(esfExternos).getArray()[0][0] / mRigidezTransformada(estacas).getArray()[0][0];
        mVL[1][0] = 0;
        mVL[2][0] = 0;
        mVL[3][0] = 0;
        mVL[4][0] = esfExternosTransformados(esfExternos).getArray()[4][0] / mRigidezTransformada(estacas).getArray()[4][4];
        mVL[5][0] = esfExternosTransformados(esfExternos).getArray()[5][0] / mRigidezTransformada(estacas).getArray()[5][5];

        return new Matrix(mVL);
    }

    private Matrix mComponentesDeVetorTransformada(Estaca[] estacas) {

        return tensorTransformacao().times(mComponentesDeVetor(estacas));
    }

}
